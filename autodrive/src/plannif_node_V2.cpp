#include "../include/plannif_node_V2.hpp"

/*
	- à chaque fois que la map change de taille tout re-init (fait dans mapCallback + initMap)		V

	- à chaque fois qu'on reçoit une update de la carte :

		- enregistrer la map dans map_data		) dans map_callback									V
		- changer les -1 en 0					)													V

		- faire une fermeture de map_data		) Dans ouverture									V
			- dilatation   map_data -> mapData_temp
			- erosion   mapData_temp -> map_data

		- calculer goalMap						) calculGoalMap
			- diminuer la résolution : map_data -> goalMap	) map2Goal								V
			- appliquer recherche en largeur				) rechLarg								V
			- renormaliser									) normalise								V
			- convolution avec un filtre gaussien			) applyConvolution
		
		- envoyer et print goalMap				) sendStatic										V
*/


PlannifNode_V2::PlannifNode_V2() : tfBuffer(), tfListener(tfBuffer){
	ROS_INFO("PlannifNode_V2::PlannifNode_V2()\n");

	originMap.position.x = -20;
	originMap.position.y = -10;

	// On dit que la carte est vide et on initialise dim pour que ce soit lisible
	staticPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
	staticPotential.layout.data_offset = 0;
	staticPotential.layout.dim[0].label = "height";
	staticPotential.layout.dim[0].size = 0;
	staticPotential.layout.dim[0].stride = 0;
	staticPotential.layout.dim[1].label = "width";
	staticPotential.layout.dim[1].size = 0;
	staticPotential.layout.dim[1].stride = 0;
	
	// Calcul des filtres parce qu'en vrai leurs valeurs sont constantes
	gaussianDelta = TAILLE_FILTRE_GAUSS/2;
	preCalculateFilter(gaussianFilter, gaussianDelta, TAILLE_FILTRE_GAUSS, WALL_MULT);

	//Init Sub, Pub et timer
	pub_staticmap = nh_.advertise<std_msgs::UInt8MultiArray>("stc_pot", 1000);
	
	sub_goal_ = nh_.subscribe("my_goal", 1, &PlannifNode_V2::goalCallback, this);
	sub_activation_ = nh_.subscribe("rbt_actv", 1, &PlannifNode_V2::changeState, this);
}

PlannifNode_V2::~PlannifNode_V2() {
	ROS_INFO("PlannifNode_V2::~PlannifNode_V2()\n");
	// Je crois que normalement les vecteurs sont vidés à la destruction des objets
	// mais je fais ça pour m'en assurer.
	staticPotential.data.clear();
	staticPotential.data.resize(0);
	staticPotential.layout.dim.resize(0);

	delete[] map_data;
	delete[] mapData_temp;
	delete[] goalMap;
	delete[] ptsToChange;

	sub_activation_.shutdown();
	sub_goal_.shutdown();
	sub_map_.shutdown();
	// send_static_timer.stop();
}

void PlannifNode_V2::changeState(const std_msgs::Bool::ConstPtr& stop){
	ROS_INFO("PLANNIF : changeState %d et %d", stop->data, first_init);
	if(stop->data){
		//Goal atteint
		//Eteindre les topics et les timers requis
		sub_map_.shutdown();
		// send_static_timer.stop();
		
		first_init = true;
	}
	//Seulement si on rallume la node
	else if(first_init){
		sub_map_ = nh_.subscribe("/gmap", 1, &PlannifNode_V2::mapCallback, this);
	}
}

void PlannifNode_V2::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	og_goal_point[0] = msg->pose.position.x;
	og_goal_point[1] = msg->pose.position.y;
	
	pastPositionBrut.resize(0);
	pastPosition.resize(0);
	
	if (!first_init) { // On a la carte donc l'origine de la carte
		goalOffset();
		//Recalcul map
		calculGoalMap();
		sendStaticPotential(staticPotential);
	}
}

void PlannifNode_V2::goalOffset() {
	goal_point[0] = (og_goal_point[0] - originMap.position.x)/resolution;
	goal_point[1] = staticPotential.layout.dim[0].size -1 - (og_goal_point[1] - originMap.position.y)/resolution;
	ROS_INFO("PlannifNode_V2::goalOffset() : goal_point = [%f, %f]", goal_point[0], goal_point[1]);
}

void PlannifNode_V2::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	ROS_INFO("PlannifNode_V2::mapCallback(msg), msg->taille(w,h) = (%d, %d)",
		msg->info.width,
		msg->info.height
	);
	
	//Lors de la première initialisation
	if(first_init){
		originMap = msg->info.origin;
		resolution = msg->info.resolution;
		initMaps(msg->info.width, msg->info.height);

		//Recalcul du goal
		goalOffset();
		
		// send_static_timer = nh_.createTimer(ros::Duration(1.0f/SEND_MAP_HZ), &PlannifNode_V2::sendStaticPotential, this);
		first_init = false;
	}

	//Si la taille de la map change
	if(staticPotential.layout.dim[0].size != msg->info.height
	|| staticPotential.layout.dim[1].size != msg->info.width){
		//Initialise toutes les données
		ROS_INFO("PLANNIF : changement de taille de gmap. taille = [%d, %d]", msg->info.width, msg->info.height);
		//Récupére l'origine de la map (en m)
		originMap = msg->info.origin;
		resolution = msg->info.resolution;
		initMaps(msg->info.width, msg->info.height);
		goalOffset();
		
		//Calcul des nouvelles positions précédentes
		for(uint16_t i=0; i < pastPosition.size(); i++){
			pastPosition[i][0] = (pastPositionBrut[i][0] - originMap.position.x)/resolution;
			pastPosition[i][1] = staticPotential.layout.dim[0].size -1 - (pastPositionBrut[i][1] - originMap.position.y)/resolution;
		}
	}

	//Copie la map reçue dans les données à traiter
	const std::vector<signed char>& data_vector = msg->data;
	delete[] map_data;
	map_data = new uint8_t[data_vector.size()];
	std::copy(data_vector.begin(), data_vector.end(), map_data);
	delete[] mapData_temp;
	mapData_temp = new uint8_t[data_vector.size()];

	//calcul du Potentiel
	calculGoalMap();
	sendStaticPotential(staticPotential);
}


void PlannifNode_V2::initMaps(uint32_t width_, uint32_t height_){
	ROS_INFO("PlannifNode_V2::initMaps(width = %d,  height = %d)\n",
	 	width_, height_
	);
	uint64_t potentialMapSize = width_ * height_;
	uint64_t goalMapSize = potentialMapSize / goalMapRes*goalMapRes;

	//staticPotential
	staticPotential.layout.dim[0].size = height_;
	staticPotential.layout.dim[1].size = width_;
	staticPotential.data.resize(potentialMapSize * sizeof(uint8_t));
	std::fill(staticPotential.data.begin(), staticPotential.data.end(), 0);

	delete[] goalMap;
	goalMap = new float[goalMapSize];
	ROS_INFO("goalMap cree taille : %ld", goalMapSize);
	std::fill(goalMap, goalMap + goalMapSize, 0);

	delete[] ptsToChange;
	ptsToChangeSize = 4* goalMapSize;
	ptsToChange = new uint32_t[ptsToChangeSize];
	ROS_INFO("taille de ptsToChange : %ld", ptsToChangeSize);
}


void PlannifNode_V2::calculGoalMap() {
	// ROS_INFO("PlannifNode_V2::calculGoalMap()");
	uint32_t height = staticPotential.layout.dim[0].size;
	uint32_t width = staticPotential.layout.dim[1].size;

	// Changer les -1 de msg en 0 dans map_data
	for (uint64_t i=0; i<height * width; i++) {
		map_data[i] = map_data[i] == 255 ? 0 : map_data[i];
	}

	fermeture(map_data, width, height, 2); // -> map_data

	map2Goal(height, width); // -> goalMap
	rechLarg(height, width); // -> goalMap
	normalise(height, width); // -> mapData_temp

	applyConvolution(mapData_temp, height, width, gaussianFilter, gaussianDelta); // -> staticPotential

	//Affichage des maps
	printMap(staticPotential, "staticPotential", 255);
}

/**
* Transpose map_data dans goalMap en :
*	symetrisant la carte selon y
*	changeant les 100 (mur) en -1
*	changeant les autres    en INFINITY
*/
void PlannifNode_V2::map2Goal(uint32_t height, uint32_t width) {
	// ROS_INFO("PlannifNode_V2::map2Goal");
	uint32_t h = height / goalMapRes;
	uint32_t w = width / goalMapRes;
	bool contientMur;
	for (uint32_t y=0; y<h; y++) {
		for (uint32_t x=0; x<w; x++) {
			contientMur = false;
			for (uint8_t dy=0; dy < goalMapRes; dy++) {
				for (uint8_t dx=0; dx < goalMapRes; dx++) {
					if (map_data[(height-1 - (goalMapRes*y + dy))*width + goalMapRes*x + dx] > 84) {
						contientMur = true;
						dy = 254;
						break;
					}
				}
			}
			if (contientMur) {goalMap[y*w + x] = -1;}
			else {goalMap[y*w + x] = INFINITY;}
			//ROS_INFO("goalMap[%d, %d] = %f", x, y, goalMap[y*w +x]);
		}
	}
}



void PlannifNode_V2::rechLarg(uint32_t height, uint32_t width) {
	ROS_INFO("PlannifNode_V2::rechLarg");
	uint32_t h = height / goalMapRes;
	uint32_t w = width / goalMapRes;
	
	uint32_t x, y;
	uint32_t pXd, pYd;
	float sqrt2 = sqrt(2);
	float temp;

	uint64_t curseur = 0;
	uint64_t end = 2;

	int64_t goalX = round(goal_point[0] / goalMapRes);
	int64_t goalY = round(goal_point[1] / goalMapRes);

	if (goalX >= w) goalX = w-1;
	else if (goalX < 0) goalX = 0;
	if (goalY >= h) goalY = h-1;
	else if (goalY < 0) goalY = 0;

	ptsToChange[curseur   ] = goalX;
	ptsToChange[curseur +1] = goalY;

	goalMap[(uint64_t)(ptsToChange[curseur +1]*w + ptsToChange[curseur])] = 0;

	while (curseur < end && end < ptsToChangeSize) {
		// ROS_INFO("curseur : %ld,   end : %ld", curseur, end);
		x = ptsToChange[curseur];
		y = ptsToChange[curseur+1];
		// ROS_INFO("goalMap[%d, %d] = %f", x, y, goalMap[y*w+x]);
		for (int8_t dx=-1; dx < 2; dx++) {
			pXd = x+dx;
			if (pXd < w) {
				for (int8_t dy=-1; dy < 2; dy++) {
					pYd = y+dy;
					if (pYd < h && goalMap[pYd*w + pXd] >= 0) { // Dans la goalMap et pixel ne contient pas de mur
						// ROS_INFO("\tgoalMap[%d, %d], indice = %d", pXd, pYd, pYd*w+pXd);

						temp = goalMap[y*w + x];
						if (dx != 0 && dy != 0) temp += sqrt2;
						else temp += 1;
					
						// ROS_INFO("\ttemp = %f et goalMap[%d, %d] = %f", temp, pXd, pYd, goalMap[pYd*w + pXd]);
						//ROS_INFO("%d", (uint32_t)goalMap[pYd*w + pXd]);
						if (temp < goalMap[pYd*w + pXd]) {
							goalMap[pYd*w + pXd] = temp;
							ptsToChange[end   ] = pXd;
							ptsToChange[end +1] = pYd;
							end += 2;
						}
					}
				}
			}
		}
		curseur += 2;
	}
}


/**
* Normalise goalMap pour le remettre dans mapData_temp
*/
void PlannifNode_V2::normalise(uint32_t height, uint32_t width) {
	// ROS_INFO("PlannifNode_V2::normalise");
	uint32_t h = height / goalMapRes;
	uint32_t w = width / goalMapRes;

	try {
        tfRobot2Map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        actualPosition[0] = tfRobot2Map.transform.translation.x;
        actualPosition[1] = tfRobot2Map.transform.translation.y;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
    }
	actualPosition[0] = (actualPosition[0] - originMap.position.x)/resolution;
    actualPosition[1] = height -1 - (actualPosition[1] - originMap.position.y)/resolution;


	float max = goalMap[(uint32_t)round(actualPosition[1]/goalMapRes)* w + (uint32_t)round(actualPosition[0]/goalMapRes)] + 20;

	float temp;
	for (uint32_t y=0; y<h; y++) {
		for (uint32_t x=0; x<w; x++) {
			if (goalMap[y*w + x] != -1) { // Pas un mur
				temp = round(goalMap[y*w + x] * GOAL_VAL_MAX / max);
				temp = temp > GOAL_VAL_MAX ? GOAL_VAL_MAX : temp;
				for (uint8_t dy=0; dy < goalMapRes; dy++) {
					for (uint8_t dx=0; dx < goalMapRes; dx++) {

						mapData_temp[(goalMapRes*y + dy)*width + goalMapRes*x + dx] = temp;

						// ROS_INFO("staticPotential[%d, %d] = %d", x+dx, y+dy, staticPotential.data[(goalMapRes*y + dy)*staticPotential.layout.dim[1].size + goalMapRes*x + dx]);
					}
				}
			}
			else { // Il y a un mur
				for (uint8_t dy=0; dy < goalMapRes; dy++) {
					for (uint8_t dx=0; dx < goalMapRes; dx++) {
						mapData_temp[(goalMapRes*y + dy)*width + goalMapRes*x + dx] = 255;

						// ROS_INFO("staticPotential[%d, %d] = %d", x+dx, y+dy, staticPotential.data[(goalMapRes*y + dy)*staticPotential.layout.dim[1].size + goalMapRes*x + dx]);
					}
				}
			}
		}
	}
}


void PlannifNode_V2::sendStaticPotential(const std_msgs::UInt8MultiArray& msg){
	// ROS_INFO("PlannifNode_V2::sendStaticPotential()\n");

	//Envoyer la map sur le topic
	pub_staticmap.publish(msg);
	// std::fill(staticPotential.data.begin(), staticPotential.data.end(), 0);
}




/**
* @bug Il y a un dépassement de mémoire si taille_filtre est pair ou si le filtre n'a pas la taille indiquée.
*/
void PlannifNode_V2::preCalculateFilter(float* filter_, uint8_t delta, int taille_filtre, float mult){
	// ROS_INFO("PlannifNode_V2::preCalculateFilter(filter, delta=%d, taille_filtre=%d, mult=%f)\n",
	// 	delta, taille_filtre, mult
	// );
	float distance_centre = 0;
	float somme=0;
	float sigma = -4 * log(0.01f)/(taille_filtre*taille_filtre);
	printf("\n");
	for(int16_t y=-delta; y<delta+1; y++){
		for(int16_t x=-delta; x<delta+1; x++){

			distance_centre = abs(x*x + y*y); // distance carrée
			filter_[x+delta + (y+delta)*taille_filtre] = exp(-sigma*distance_centre);
			somme += filter_[x+delta + (y+delta)*taille_filtre];
		}
	}
	float rapport = mult / somme;
	for (uint16_t i=0; i < taille_filtre*taille_filtre; i++) {
		filter_[i] *= rapport;
	}
}


void PlannifNode_V2::applyConvolution(uint8_t* mapData, uint32_t height, uint32_t width, float* filter_, uint8_t delta){
	// ROS_INFO("PlannifNode_V2::applyConvolution(mapData = %p, height=%d, width=%d, filter_=%p, delta=%d)\n",
	// 	mapData, height, width, filter_, delta
	// );

	float temp;
	uint16_t i;
	uint64_t pYdW;
	for (uint32_t y=delta; y < height - delta; y++) {
		for (uint32_t x=delta; x < width - delta; x++) {
			temp = 0;
			i = 0;
			for(int16_t dy=-delta; dy < delta+1; dy++){
				pYdW = (y+dy)*width;
				for(int16_t dx=-delta; dx < delta+1; dx++){
					temp += mapData[pYdW + x+dx] * filter_[i];
					i++;
				}
			}
			
			if (temp < 0) temp = 0;
			else if (temp > 255) temp = 255;
			
			staticPotential.data[y*width + x] = (uint8_t)round(temp);		
			// staticPotential.data[y*width + x] = mapData_temp[y*width+x];		
		}
	}
}
  

/**
* Cette fonction utilise mapData_temp. Merci de ne pas utiliser mapData_temp
*/
void PlannifNode_V2::fermeture(uint8_t* mapData, uint32_t width, uint32_t height, uint8_t taille) {
	ROS_INFO("PlannifNode_V2::fermeture(mapData=%p, width=%d, height=%d, taille=%d)",
	mapData, width, height, taille);
	uint8_t* tabs[2] = {mapData, mapData_temp};
	bool sens = 0;

	// Dilatation
	uint8_t max;
	uint32_t pYd, pXd;
	for (uint8_t i=0; i<taille; i++) {
		for (uint32_t y=1; y < height-1; y++) {
			for (uint32_t x=1; x < width-1; x++) {
				max = 0;
				for (int8_t dy=-1; dy < 2; dy++) {
					pYd = y + dy;
					for (int8_t dx=-1; dx < 2; dx++) {
						pXd = x + dx;
						if (max < tabs[sens][pYd * width + pXd]) { // il y a un pixel plus probable d'être un mur
							max = tabs[sens][pYd * width + pXd];
						}
					}
				}
				tabs[!sens][y*width + x] = max;
			}
		}
		sens = !sens; // inversion du sens
	}

	// Erosion
	uint8_t min;
	for (uint8_t i=0; i<taille; i++) {
		for (uint32_t y=1; y < height-1; y++) {
			for (uint32_t x=1; x < width-1; x++) {
				min = 255;
				for (int8_t dy=-1; dy < 2; dy++) {
					pYd = y + dy;
					for (int8_t dx=-1; dx < 2; dx++) {
						pXd = x + dx;
						if (min > tabs[sens][pYd * width + pXd]) { // il y a un pixel moins probable d'être un mur
							min = tabs[sens][pYd * width + pXd];
						}
					}
				}
				tabs[!sens][y*width + x] = min;
			}
		}
		sens = !sens; // inversion du sens
	}
}

void PlannifNode_V2::printMap(std_msgs::UInt8MultiArray& map, std::string nom, uint16_t valMax) {
	ROS_INFO("PlannifNode_V2::printMap<UInt8MultiArray>(map, nom)\n");
	// Ouverture du fichier
	std::ofstream wf("/home/catkin_ws/src/autodrive/mapdump/" + nom + ".pgm", std::ios::out | std::ios::binary | std::ios::trunc);
	if (!wf) {
			ROS_INFO("Cannot open file !");
			return;
	}
	
	// Création de la métadata du pgm
	char metaData[128] = "P5\n";
	
	size_t metaDataSize = 3;

	uint32_t width = map.layout.dim[1].size;
	uint32_t height = map.layout.dim[0].size;
	if (width && height) {
		char tampon[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		uint8_t tamponSize = 0;

		// Récupération des métadonnées sous forme ASCII avec retours de ligne
		uint64_t dec=1;
		while (dec <= width) {
			tampon[tamponSize++] = '0' + (width % (dec * 10))/ dec;
			dec *= 10;
		}
		do {
			metaData[metaDataSize++] = tampon[--tamponSize];
		} while (tamponSize > 0);
		
		metaData[metaDataSize++] = ' ';
		
		dec = 1;
		while (dec <= height) {
			tampon[tamponSize++] = '0' + (height % (dec * 10))/ dec;
			dec *= 10;
		}
		do {
			metaData[metaDataSize++] = tampon[--tamponSize];
		} while (tamponSize > 0);
		
		metaData[metaDataSize++] = '\n';
		
		dec = 1;
		while (dec <= valMax) {
			tampon[tamponSize++] = '0' + (valMax % (dec * 10))/ dec;
			dec *= 10;
		}
		do {
			metaData[metaDataSize++] = tampon[--tamponSize];
		} while (tamponSize > 0);

		metaData[metaDataSize++] = '\n';

		// impression de la métadata dans le fichier
		wf.write((char*)metaData, metaDataSize);
		
		// Impression de la carte dans le fichier
		wf.write((char*)map.data.data(), (size_t)(width*height));
	}
	// ROS_INFO("PlannifNode_V2::printMap ~%d ko ecrits dans /home/catkin_ws/src/autodrive/mapdump/", width*height/1000);
}

void PlannifNode_V2::printMap(uint8_t* map, uint32_t height, uint32_t width, std::string nom, uint16_t valMax) {
	ROS_INFO("PlannifNode_V2::printMap<uint8_t>(map, nom)\n");
	// Ouverture du fichier
	std::ofstream wf("/home/catkin_ws/src/autodrive/mapdump/" + nom + ".pgm", std::ios::out | std::ios::binary | std::ios::trunc);
	if (!wf) {
			ROS_INFO("Cannot open file !");
			return;
	}
	
	// Création de la métadata du pgm
	char metaData[128] = "P5\n";
	
	size_t metaDataSize = 3;

	if (width && height) {
		char tampon[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		uint8_t tamponSize = 0;

		// Récupération des métadonnées sous forme ASCII avec retours de ligne
		uint64_t dec=1;
		while (dec <= width) {
			tampon[tamponSize++] = '0' + (width % (dec * 10))/ dec;
			dec *= 10;
		}
		do {
			metaData[metaDataSize++] = tampon[--tamponSize];
		} while (tamponSize > 0);
		
		metaData[metaDataSize++] = ' ';
		
		dec = 1;
		while (dec <= height) {
			tampon[tamponSize++] = '0' + (height % (dec * 10))/ dec;
			dec *= 10;
		}
		do {
			metaData[metaDataSize++] = tampon[--tamponSize];
		} while (tamponSize > 0);
		
		metaData[metaDataSize++] = '\n';
		
		dec = 1;
		while (dec <= valMax) {
			tampon[tamponSize++] = '0' + (valMax % (dec * 10))/ dec;
			dec *= 10;
		}
		do {
			metaData[metaDataSize++] = tampon[--tamponSize];
		} while (tamponSize > 0);

		metaData[metaDataSize++] = '\n';

		// impression de la métadata dans le fichier
		wf.write((char*)metaData, metaDataSize);
		
		// Impression de la carte dans le fichier
		wf.write((char*)map, (size_t)(width*height));
	}
	// ROS_INFO("PlannifNode_V2::printMap ~%d ko ecrits dans /home/catkin_ws/src/autodrive/mapdump/", width*height/1000);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "plannif_node_V2");
	PlannifNode_V2 node;

	ros::spin();
	return 0;
}

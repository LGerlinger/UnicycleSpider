#include "../include/plannif_node.hpp"


// Chercher les print de debuggage du memory leak : 
// MEMORY LEAK DEBUGGING

PlannifNode::PlannifNode() : tfBuffer(), tfListener(tfBuffer){
	ROS_INFO("PlannifNode::PlannifNode()\n");

    originMap.position.x = -20;
    originMap.position.y = -10;

    /* // sera utilisé une fois que le problème mémoire sera résolu (22 jan 2024)
    std_msgs::UInt8MultiArray& tabMap[4] = {
        initPotential,
        goalPotential,
        tracePotential,
        staticPotential
    }

    // On dit que les cartes sont vides et on initialise dim pour que ce soit lisible
    for (uint8_t i=0; i < 4 i++) {
        tabMap[i].layout.dim.resize(2, std_msgs::MultiArrayDimension());
        tabMap[i].layout.data_offset = 0;
        tabMap[i].layout.dim[0].label = "height";
        tabMap[i].layout.dim[0].size = 0;
        tabMap[i].layout.dim[0].stride = 0;
        tabMap[i].layout.dim[1].label = "width";
        tabMap[i].layout.dim[1].size = 0;
        tabMap[i].layout.dim[1].stride = 0;
    }*/

    // On dit que les cartes sont vides et on initialise dim pour que ce soit lisible
    initPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    initPotential.layout.data_offset = 0;
    initPotential.layout.dim[0].label = "height";
    initPotential.layout.dim[0].size = 0;
    initPotential.layout.dim[0].stride = 0;
    initPotential.layout.dim[1].label = "width";
    initPotential.layout.dim[1].size = 0;
    initPotential.layout.dim[1].stride = 0;
    
    goalPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    goalPotential.layout.data_offset = 0;
    goalPotential.layout.dim[0].label = "height";
    goalPotential.layout.dim[0].size = 0;
    goalPotential.layout.dim[0].stride = 0;
    goalPotential.layout.dim[1].label = "width";
    goalPotential.layout.dim[1].size = 0;
    goalPotential.layout.dim[1].stride = 0;
    
    tracePotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    tracePotential.layout.data_offset = 0;
    tracePotential.layout.dim[0].label = "height";
    tracePotential.layout.dim[0].size = 0;
    tracePotential.layout.dim[0].stride = 0;
    tracePotential.layout.dim[1].label = "width";
    tracePotential.layout.dim[1].size = 0;
    tracePotential.layout.dim[1].stride = 0;
    
    staticPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    staticPotential.layout.data_offset = 0;
    staticPotential.layout.dim[0].label = "height";
    staticPotential.layout.dim[0].size = 0;
    staticPotential.layout.dim[0].stride = 0;
    staticPotential.layout.dim[1].label = "width";
    staticPotential.layout.dim[1].size = 0;
    staticPotential.layout.dim[1].stride = 0;
    
    // Calcul des filtres parce qu'en vrai leurs valeurs sont constantes
    wallDelta = TAILLE_FILTRE_WALL/2;
    preCalculateFilter(wallFilter, wallDelta, TAILLE_FILTRE_WALL, WALL_MULT);

    traceDelta = TAILLE_FILTRE_TRACE/2;
    preCalculateFilter(traceFilter, traceDelta, TAILLE_FILTRE_TRACE, TRACE_MULT);

    //Init Sub, Pub et timer
    pub_staticmap = nh_.advertise<std_msgs::UInt8MultiArray>("stc_pot", 1000);
    
    sub_goal_ = nh_.subscribe("my_goal", 1, &PlannifNode::goalCallback, this);
    sub_activation_ = nh_.subscribe("rbt_actv", 1, &PlannifNode::changeState, this);
}

PlannifNode::~PlannifNode() {
    ROS_INFO("PlannifNode::~PlannifNode()\n");
    // Je crois que normalement les vecteurs sont vidés à la destruction des objets
    // mais je fais ça pour m'en assurer.
    initPotential.data.clear();
    initPotential.data.resize(0);
    initPotential.layout.dim.resize(0);

    goalPotential.data.clear();
    goalPotential.data.resize(0);
    goalPotential.layout.dim.resize(0);

    tracePotential.data.clear();
    tracePotential.data.resize(0);
    tracePotential.layout.dim.resize(0);

    staticPotential.data.clear();
    staticPotential.data.resize(0);
    staticPotential.layout.dim.resize(0);

    delete[] map_data;

    sub_activation_.shutdown();
    sub_goal_.shutdown();
    sub_map_.shutdown();
    send_static_timer.stop();
}

void PlannifNode::changeState(const std_msgs::Bool::ConstPtr& stop){
    // ROS_INFO("PLANNIF : changeState %d et %d", stop->data, first_init);
    if(stop->data){
        //Goal atteint
        //Eteindre les topics et les timers requis
        sub_map_.shutdown();
        send_static_timer.stop();
        
        goal_point[0] = INFINITY;
        goal_point[1] = INFINITY;
        
        first_init = true;
    }
    //Seulement si on rallume la node
    else if(first_init){
        sub_map_ = nh_.subscribe("/gmap", 1, &PlannifNode::mapCallback, this);
    }
}

void PlannifNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal_point[0] = msg->pose.position.x;
    goal_point[1] = msg->pose.position.y;
    
    if (!first_init) { // On a la carte donc l'origine de la carte
        goalOffset();
        
        //Recalcul map 
        calculGoalPotential();
    }
}

void PlannifNode::goalOffset() {
    goal_point[0] = (goal_point[0] - originMap.position.x)/resolution;
    goal_point[1] = goalPotential.layout.dim[0].size -1 - (goal_point[1] - originMap.position.y)/resolution;
    ROS_INFO("PlannifNode::goalOffset() : goal_point = [%f, %f]", goal_point[0], goal_point[1]);
}

void PlannifNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	// ROS_INFO("PlannifNode::mapCallback(msg), msg->taille(w,h) = (%d, %d)\n",
	// 	msg->info.width,
	// 	msg->info.height
	// );
    if(first_init
    || staticPotential.layout.dim[0].size != msg->info.height
    || staticPotential.layout.dim[1].size != msg->info.width){
        //Initialise toutes les données
        ROS_INFO("PLANNIF : mapCallback first init ou changement de taille de gmap");
        //Récupére l'origine de la map (en m)
        originMap = msg->info.origin;
        resolution = msg->info.resolution;

        initMaps(msg->info.width, msg->info.height);

        if (!isinf(goal_point[0])) { // Si on a recu l'objectif à l'avance
            ROS_INFO("goal est bien INFINITY");
            goalOffset();
            calculGoalPotential();
        }
        
        send_static_timer = nh_.createTimer(ros::Duration(1.0f/SEND_MAP_HZ), &PlannifNode::sendStaticPotential, this);
        first_init = false;
    }

    //Copie la map reçue dans les données à traiter
    const std::vector<signed char>& data_vector = msg->data;
    delete[] map_data;
    map_data = new uint8_t[data_vector.size()];
    std::copy(data_vector.begin(), data_vector.end(), map_data);
    ROS_INFO("map_data cree taille : %ld.  on lui passe %ld", data_vector.end() - data_vector.begin(), data_vector.size());
    ROS_INFO("PlannifNode::mapCallback taille nouvelle gmap : [%d, %d]", msg->info.width, msg->info.height);
    //calculduPotentiel
    calculInitPotential();
}

/**
* @bug crash lorsque la map s'aggrandie :( jsp pk
ça a l'air de crasher parce qu'on veut augmenter la taille du vector initPotential.data
une piste possible est qu'on écrit sur de la mémoire qui n'appartient pas au vecteur jsp comment ni quand
*/
void PlannifNode::initMaps(uint32_t width_, uint32_t height_){
	ROS_INFO("PlannifNode::initMaps(width = %d,  height = %d)\n",
	 	width_, height_
	);
    size_t totalSize = width_ * height_ * sizeof(uint8_t);
	
    //InitPotential
    initPotential.layout.dim[0].size = height_;
    initPotential.layout.dim[1].size = width_;
    //initPotential.data.clear();
    ROS_INFO("initPotential.data : size = %ld", initPotential.data.size());
    ROS_INFO("resize va crasher pour totalSize=%ld", totalSize);
    initPotential.data.resize(totalSize);
    ROS_INFO("2");
    std::fill(initPotential.data.begin(), initPotential.data.end(), 0);
    ROS_INFO("initPotential");
    
    //goalPotential
    goalPotential.layout.dim[0].size = height_;
    goalPotential.layout.dim[1].size = width_;
    //goalPotential.data.clear();
    goalPotential.data.resize(totalSize);
    ROS_INFO("goalPotential");

    //tracePotential
    tracePotential.layout.dim[0].size = height_;
    tracePotential.layout.dim[1].size = width_;
    //tracePotential.data.clear();
    tracePotential.data.resize(totalSize);
    std::fill(tracePotential.data.begin(), initPotential.data.end(), 0);
    ROS_INFO("tracePotential");

    //staticPotential
    staticPotential.layout.dim[0].size = height_;
    staticPotential.layout.dim[1].size = width_;
    //staticPotential.data.clear();
    staticPotential.data.resize(totalSize);
    std::fill(staticPotential.data.begin(), initPotential.data.end(), 0);
    ROS_INFO("staticPotential");
    
    ROS_INFO("PlannifNode::initMaps end");
}

/**
* @bug Il y a un dépassement de mémoire si taille_filtre est pair ou si le filtre n'a pas la taille indiquée.
*/
void PlannifNode::preCalculateFilter(float* filter_, uint8_t delta, int taille_filtre, float mult){
	ROS_INFO("PlannifNode::preCalculateFilter(filter, delta=%d, taille_filtre=%d, mult=%f)\n",
		delta, taille_filtre, mult
	);
    float distance_centre = 0;
		float somme=0;
		float sigma = -4 * log(0.01f)/(taille_filtre*taille_filtre);
    printf("\n");
    for(int y=-delta; y<delta+1; y++){
        for(int x=-delta; x<delta+1; x++){
            // MEMORY LEAK DEBUGGING
            if (x+delta + (y+delta)*taille_filtre < 0 || taille_filtre*taille_filtre <= x+delta + (y+delta)*taille_filtre) {
                ROS_INFO("PlannifNode::preCalculateFilter, MEMORY LEAK!! : %d", x+delta + (y+delta)*taille_filtre );
            }
            // END MEMORY LEAK DEBUGGING

            distance_centre = abs(x*x + y*y); // distance carrée
            filter_[x+delta + (y+delta)*taille_filtre] = exp(-sigma*distance_centre);
            somme += filter_[x+delta + (y+delta)*taille_filtre];
        }
    }
    float rapport = (float)(mult * GOAL_VAL_MAX) / (OCCUPANCYGRID_VAL_MAX * somme);
		for (uint16_t i=0; i < taille_filtre*taille_filtre; i++) {
            // MEMORY LEAK DEBUGGING
            if (i < 0 || taille_filtre*taille_filtre <= i) {
                ROS_INFO("PlannifNode::preCalculateFilter 2, MEMORY LEAK!! : %d", i );
            }
            // END MEMORY LEAK DEBUGGING
				filter_[i] *= rapport;
		}
}

/**
* @param coef : valeur du pixel indice de l'image à copier/filtrer
*/
void PlannifNode::applyFilter(std_msgs::UInt8MultiArray* mapPotential, float* filter_, int indice, uint8_t delta, int coef){
	// ROS_INFO("PlannifNode::applyFilter(mapPotential = %p, filter_, indice=%d,  delta=%d)\n",
	// 	mapPotential, indice, delta
	// );
    uint16_t i = 0;
    int ligne = 0;
    for(int16_t y=-delta; y<delta+1; y++){
        ligne = y*(mapPotential->layout.dim[1].size);
        for(int16_t x=-delta; x<delta+1; x++){
            // MEMORY LEAK DEBUGGING
            if (indice + x + ligne < 0 || mapPotential->layout.dim[0].size * mapPotential->layout.dim[1].size <= indice + x + ligne) {
                ROS_INFO("PlannifNode::applyFilter, MEMORY LEAK!! : %d", indice + x + ligne);
            }
            // END MEMORY LEAK DEBUGGING
            // MEMORY LEAK DEBUGGING
            if (i < 0 || (2*delta+1) * (2*delta+1) <= i) {
                ROS_INFO("PlannifNode::applyFilter 2, MEMORY LEAK!! : %d", i);
            }
            // END MEMORY LEAK DEBUGGING
            mapPotential->data[indice + x + ligne] += filter_[i] * coef;
            i++;
        }
    }
}

/**
* @brief Presque pareil que applyFilter mais de map_data vers initPotential
*/
void PlannifNode::applyConvolution(uint8_t* mapData, uint16_t width, float* filter_, uint64_t indice, uint8_t delta, uint8_t& ptToChange){
	//ROS_INFO("PlannifNode::applyConvolution(mapData = %p, width = %d, filter_ = %p, indice=%ld,  delta=%d,  ptToChange=%d)\n",
	//	mapData, width, filter_, indice, delta, ptToChange
	//);
    uint16_t i = 0;
    int ligne = 0;
    float temp = 0;
    for(int16_t y=-delta; y<delta+1; y++){
        ligne = y*width;
        for(int16_t x=-delta; x<delta+1; x++){
            // MEMORY LEAK DEBUGGING
            if (indice + x + ligne < 0 || initPotential.layout.dim[0].size * initPotential.layout.dim[1].size <= indice + x + ligne) {
                ROS_INFO("PlannifNode::applyConvolution, MEMORY LEAK!! : %d", i);
            }
            // END MEMORY LEAK DEBUGGING
            // MEMORY LEAK DEBUGGING
            if (i < 0 || (2*delta+1) * (2*delta+1) <= i) {
                ROS_INFO("PlannifNode::applyConvolution 2, MEMORY LEAK!! : %d", i);
            }
            // END MEMORY LEAK DEBUGGING
            temp += mapData[indice + x + ligne] * filter_[i];
            i++;
        }
    }
    
    if (temp < 0) temp = 0;
    else if (temp > GOAL_VAL_MAX) temp = GOAL_VAL_MAX;
    
    //ptToChange = (uint8_t)round(temp);
}


void PlannifNode::calculInitPotential(){
	ROS_INFO("PlannifNode::calculInitPotential(), wallDelta=%d\n", wallDelta);
	
	uint32_t h = initPotential.layout.dim[0].size;
	uint32_t w = initPotential.layout.dim[1].size;
	
    uint64_t indice;
    for (indice=0; indice < h*w; indice++) {
        if (map_data[indice] > 101) { // La zone est inconnue
            map_data[indice] = 0;
        }
    }

	for(uint32_t y=wallDelta; y<(h-wallDelta); y++){
		for(uint32_t x=wallDelta; x<(w-wallDelta); x++){
			indice = x + y*w;
			//ROS_INFO("map = %ld\t", indice, x + (h-y-1)*w);
			//faire le filtre
			applyConvolution(map_data, w, wallFilter,
			x + (h-y-1)*w,
			wallDelta, initPotential.data[indice]);
		}
	}
	
    printMap(initPotential, "initPotential", GOAL_VAL_MAX);
}

void PlannifNode::calculGoalPotential() {
    // ROS_INFO("PlannifNode::calculGoalPotential()\n");
    int delta = 0;
    int distance_goal = 0;

    // trouver distMax
    float distMax = 0.001f;
    float dist[4];
    
    dist[0] = sqrt((goal_point[0])*(goal_point[0]) 
                 + (goal_point[1])*(goal_point[1]));

    dist[1] = sqrt((goal_point[0]-goalPotential.layout.dim[1].size)*(goal_point[0]-goalPotential.layout.dim[1].size) 
                 + (goal_point[1])*(goal_point[1]));
                 
    dist[2] = sqrt((goal_point[0])*(goal_point[0]) 
                 + (goal_point[1]-goalPotential.layout.dim[0].size)*(goal_point[1]-goalPotential.layout.dim[0].size));

    dist[3] = sqrt((goal_point[0]-goalPotential.layout.dim[1].size)*(goal_point[0]-goalPotential.layout.dim[1].size) 
                 + (goal_point[1]-goalPotential.layout.dim[0].size)*(goal_point[1]-goalPotential.layout.dim[0].size));

    for(uint8_t i=0; i<4; i++){
        if(distMax < dist[i]){
            distMax = dist[i];
        }
    }
    
    // ROS_INFO("PlannifNode::calculGoalPotential() Calcul pente %f\n", distMax);
    ROS_INFO("PlannifNode::calculGoalPotential() Goal : %f et %f\n", goal_point[0], goal_point[1]);

    //Calcul pente
    float pente = (GOAL_VAL_MAX - GOAL_VAL_MIN) / distMax;
    for(uint32_t y=0; y<(goalPotential.layout.dim[0].size); y++){
        for(uint32_t x=0; x<(goalPotential.layout.dim[1].size); x++){
            //Calcul de la distance au goal
            distance_goal = sqrt((goal_point[0]-x)*(goal_point[0]-x) 
                                        + (goal_point[1]-y)*(goal_point[1]-y));
            //Application du filtre
            goalPotential.data[x + y*goalPotential.layout.dim[1].size] = distance_goal * pente + GOAL_VAL_MIN;
        }
    }
    
    printMap(goalPotential, "goalPotential", GOAL_VAL_MAX);
}

void PlannifNode::calculTracePotential(const ros::TimerEvent& event){
	// ROS_INFO("PlannifNode::calculTracePotential(const ros::TimerEvent& event)\n");

    uint32_t indice;

    //Récupération de la position actuelle 
    try {
        tfRobot2Map = tfBuffer.lookupTransform("base_link", "map", ros::Time(0));
        actualPosition[0] = (tfRobot2Map.transform.translation.x - originMap.position.x)/resolution;
        actualPosition[1] = goalPotential.layout.dim[0].size - (tfRobot2Map.transform.translation.y - originMap.position.y)/resolution;
    }catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
    }
    

    //tracePotential
    for(uint16_t i=0; i<pastPosition.size(); i++){
        indice = pastPosition[i][0] + (pastPosition[i][1]*tracePotential.layout.dim[1].size);
        if(i == pastPosition.size()-1){
            //Si c'est le point le plus récent
            applyFilter(&tracePotential, traceFilter, indice, traceDelta, 1);
        }else{
            //Sinon on le diminue
            applyFilter(&tracePotential, traceFilter, indice, traceDelta, -1/(TAILLE_MAX_TRACE-1));
        }
    }
    
    //Met à jour le vecteur pastPositions
    if(pastPosition.size()==TAILLE_MAX_TRACE){
        pastPosition.erase(pastPosition.begin());
    }
    pastPosition.push_back({actualPosition[0], actualPosition[1]});
    
    printMap(tracePotential, "tracePotential", GOAL_VAL_MAX);
}   

void PlannifNode::sendStaticPotential(const ros::TimerEvent& event){
	// ROS_INFO("PlannifNode::sendStaticPotential()\n");

    //Faire l'addition de toutes les maps 
    addPotentialToStatic(initPotential, 1);
    addPotentialToStatic(goalPotential, 1);
    //addPotentialToStatic(tracePotential, 1);

    //Affichage des maps
    printMap(staticPotential, "staticPotential", 255);

    //Envoyer la map sur le topic
    pub_staticmap.publish(staticPotential);
    std::fill(staticPotential.data.begin(), staticPotential.data.end(), 0);
}


void PlannifNode::addPotentialToStatic(std_msgs::UInt8MultiArray& mapPotential, int coeff){
    int indice;
    for(uint32_t y=0; y<(staticPotential.layout.dim[0].size); y++){
        for(uint32_t x=0; x<(staticPotential.layout.dim[1].size); x++){
            indice = x + y*staticPotential.layout.dim[1].size;
            staticPotential.data[indice] += coeff*mapPotential.data[indice];
        }
    }
}

void PlannifNode::printMap(std_msgs::UInt8MultiArray& map, std::string nom, uint16_t valMax) {
	// ROS_INFO("PlannifNode::printMap(map, nom)\n");
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
	// ROS_INFO("PlannifNode::printMap ~%d ko ecrits dans /home/catkin_ws/src/autodrive/mapdump/", width*height/1000);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "plannif_node");
    PlannifNode node;

    ros::spin();
    return 0;
}

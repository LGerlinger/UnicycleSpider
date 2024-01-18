#include "../include/plannif_node.hpp"


PlannifNode::PlannifNode() : tfBuffer(), tfListener(tfBuffer){
	// ROS_INFO("PlannifNode::PlannifNode()\n");
    goal_point[0]=0;
    goal_point[1]=1;

    originMap.position.x = -20;
    originMap.position.y = -10;

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
    timer.stop();
    timer2.stop();
}

void PlannifNode::changeState(const std_msgs::Bool::ConstPtr& stop){
    // ROS_INFO("PLANNIF : changeState %d et %d", stop->data, first_init);
    if(stop->data){
        //Goal atteint
        //Eteindre les topics et les timers requis
        sub_map_.shutdown();
        timer.stop();
        timer2.stop();

        //Clean les maps
        std::fill(goalPotential.data.begin(), goalPotential.data.end(), 0);
        std::fill(tracePotential.data.begin(), goalPotential.data.end(), 0);
        
        first_init = true;
    }
    //Seulement si on rallume la node
    else if(first_init){
        sub_map_ = nh_.subscribe("/gmap", 1, &PlannifNode::mapCallback, this);
        timer = nh_.createTimer(ros::Duration(1.0f/CALCUL_TRACE_MAP_HZ), &PlannifNode::calculTracePotential, this);
    }
}

void PlannifNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    
    goal_point[0] = msg->pose.position.x;
    goal_point[1] = msg->pose.position.y;
    timer_goal = nh_.createTimer(ros::Duration(0.5f), &PlannifNode::checkMapInit4Goal, this);
}

void PlannifNode::checkMapInit4Goal(const ros::TimerEvent& event){
    if(first_init){
        // ROS_INFO("PLANNIF : pas de map donc pas de calcul de goal");
    }else{
        // ROS_INFO("PLANNIF : get nouveau goal");
        goal_point[0] = (goal_point[0] - originMap.position.x)/resolution;
        goal_point[1] = goalPotential.layout.dim[0].size - (goal_point[1] - originMap.position.y)/resolution -1;
        
        //Recalcul map 
        calculGoalPotential();

        //Stop le timer
        timer_goal.stop();
    }
}

void PlannifNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	// ROS_INFO("PlannifNode::mapCallback(msg), msg->taille(w,h) = (%d, %d)\n",
	// 	msg->info.width,
	// 	msg->info.height
	// );
    // ROS_INFO("PLANNIF : mapCallback");
    if(first_init){
        //Initialise toutes les données
        // ROS_INFO("PLANNIF : mapCallback first init");
        //Récupére l'origine de la map (en m)
        originMap = msg->info.origin;
        resolution = msg->info.resolution;

        initMaps(msg->info.width, msg->info.height);

        wallDelta = TAILLE_FILTRE_WALL/2;
        preCalculateFilter(wallFilter, wallDelta, TAILLE_FILTRE_WALL, WALL_COEFF_A, WALL_COEFF_B, WALL_MULT);

        traceDelta = TAILLE_FILTRE_TRACE/2;
        preCalculateFilter(traceFilter, traceDelta, TAILLE_FILTRE_TRACE, TRACE_COEFF_A, TRACE_COEFF_B, TRACE_MULT);
        
        timer2 = nh_.createTimer(ros::Duration(1.0f/SEND_MAP_HZ), &PlannifNode::sendStaticPotential, this);
        first_init = false;
    }

    //Copie la map reçu dans les données à traiter
    const std::vector<signed char>& data_vector = msg->data;
    map_data = new uint8_t[data_vector.size()];
    std::copy(data_vector.begin(), data_vector.end(), map_data);

    //calculduPotentiel
    calculInitPotential();
}

void PlannifNode::initMaps(uint32_t width_, uint32_t height_){
	// ROS_INFO("PlannifNode::initMaps(width = %d,  height = %d)\n",
	// 	width_, height_
	// );
    size_t totalSize = width_ * height_ * sizeof(uint8_t);
	
    //InitPotential
	initPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    initPotential.layout.dim[0].label = "height";
    initPotential.layout.dim[0].size = height_;
    initPotential.layout.dim[1].label = "width";
    initPotential.layout.dim[1].size = width_;
    initPotential.data.resize(totalSize);
    std::fill(initPotential.data.begin(), initPotential.data.end(), 0);

    //goalPotential
    goalPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    goalPotential.layout.dim[0].label = "height";
    goalPotential.layout.dim[0].size = height_;
    goalPotential.layout.dim[1].label = "width";
    goalPotential.layout.dim[1].size = width_;
    goalPotential.data.resize(totalSize);

    //tracePotential
    tracePotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    tracePotential.layout.dim[0].label = "height";
    tracePotential.layout.dim[0].size = height_;
    tracePotential.layout.dim[1].label = "width";
    tracePotential.layout.dim[1].size = width_;
    tracePotential.data.resize(totalSize);
    std::fill(tracePotential.data.begin(), initPotential.data.end(), 0);

    //staticPotential
    staticPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    staticPotential.layout.dim[0].label = "height";
    staticPotential.layout.dim[0].size = height_;
    staticPotential.layout.dim[1].label = "width";
    staticPotential.layout.dim[1].size = width_;
    staticPotential.data.resize(totalSize);
    std::fill(staticPotential.data.begin(), initPotential.data.end(), 0);
}

/**
* @bug Il y a un dépassement de mémoire si taille_filtre est pair.
*/
void PlannifNode::preCalculateFilter(float* filter_, uint8_t delta, int taille_filtre, float coeffA, float coeffB, float mult){
	ROS_INFO("PlannifNode::preCalculateFilter(filter, delta=%d, taille_filtre=%d,  coeffA=%f,  coeffB=%f, mult=%f)\n",
		delta, taille_filtre, coeffA, coeffB, mult
	);
    float distance_centre = 0;
    uint16_t indice=0;
		float somme=0;
    printf("\n");
    for(int y=-delta; y<delta+1; y++){
        for(int x=-delta; x<delta+1; x++){
            distance_centre = sqrt(x*x + y*y);
            filter_[x+delta + (y+delta)*taille_filtre] = 
            //filter_[indice] =
                coeffA * exp(-coeffB*distance_centre*distance_centre);
            somme += filter_[x+delta + (y+delta)*taille_filtre];
            indice++;
        }
    }
    float rapport = mult * (float)GOAL_VAL_MAX / OCCUPANCYGRID_VAL_MAX;
    ROS_INFO("double boucle passee, somme=%f,   filtre multiplie par %f", somme, rapport/somme);
    for(int y=-delta; y<delta+1; y++){
    	for(int x=-delta; x<delta+1; x++){
		  	filter_[x+delta + (y+delta)*taille_filtre] *= rapport / somme;
        printf("%f\t", filter_[x+delta + (y+delta)*taille_filtre]);
    	}
      printf("\n");
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
    for(int y=-delta; y<delta+1; y++){
        ligne = y*(mapPotential->layout.dim[1].size);
        for(int x=-delta; x<delta+1; x++){
            mapPotential->data[indice + x + ligne] += filter_[i] * coef;
            i++;
        }
    }
}

void PlannifNode::calculInitPotential(){
	// ROS_INFO("PlannifNode::calculInitPotential(), wallDelta=%d\n", wallDelta);
    int indice;
    for(uint32_t y=wallDelta; y<(initPotential.layout.dim[0].size-wallDelta); y++){
        for(uint32_t x=wallDelta; x<(initPotential.layout.dim[1].size-wallDelta); x++){
            indice = x + (y*initPotential.layout.dim[1].size);

            if(map_data[indice] < 101){
                //faire le filtre
                applyFilter(&initPotential, wallFilter,
                //  wallDelta, map_data[indice]);
x + ((initPotential.layout.dim[0].size-y-wallDelta-1)*initPotential.layout.dim[1].size),
                wallDelta, map_data[indice]);
            }
            else {
            	initPotential.data[indice] = 0;
            }
        }
    }
}

void PlannifNode::calculGoalPotential(){
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

    // ROS_INFO("PlannifNode::calculGoalPotential() END\n");
    
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
}   

void PlannifNode::sendStaticPotential(const ros::TimerEvent& event){
	// ROS_INFO("PlannifNode::sendStaticPotential()\n");

    //Faire l'addition de toutes les maps 
    addPotentialToStatic(initPotential, 1);
    addPotentialToStatic(goalPotential, 1);
    addPotentialToStatic(tracePotential, 1);

    //Affichage des maps    
    printMap(initPotential, "initPotential", GOAL_VAL_MAX);
    printMap(goalPotential, "goalPotential", GOAL_VAL_MAX);
    printMap(tracePotential, "tracePotential", GOAL_VAL_MAX);
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

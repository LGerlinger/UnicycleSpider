#include "../include/plannif_node.hpp"


PlannifNode::PlannifNode(){
	ROS_INFO("PlannifNode::PlannifNode()\n");
    goal_point[0]=0;
    goal_point[1]=1;

    pub_staticmap = nh_.advertise<nav_msgs::OccupancyGrid>("stc_pot", 1000);
    sub_map_ = nh_.subscribe("/gmap", 1, &PlannifNode::mapCallback, this);
    sub_goal_ = nh_.subscribe("move_base_simple/goal", 1, &PlannifNode::goalCallback, this);
    sub_odom_ = nh_.subscribe("/odom", 1, &PlannifNode::odomCallback, this);
    ros::Timer timer = nh_.createTimer(ros::Duration(CALCUL_TRACE_MAP_HZ), &PlannifNode::calculTracePotential, this);

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
}

void PlannifNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	// ROS_INFO("PlannifNode::odomCallback(msg), msg->position = (%f, %f)\n",
	//	msg->pose.pose.position.x,
	//	msg->pose.pose.position.y
	//);
    //MAJ actualPosition
    actualPosition[0] = msg->pose.pose.position.x;
    actualPosition[1] = msg->pose.pose.position.y;
}

void PlannifNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	ROS_INFO("PlannifNode::goalCallback -> goal : (%f, %f),   msg : (%f, %f)\n",
		goal_point[0], goal_point[1],
		msg->pose.position.x, msg->pose.position.y
	);
    if(goal_point[0] != msg->pose.position.x || goal_point[1] != msg->pose.position.y){
        goal_point[0] = msg->pose.position.x;
        goal_point[1] = msg->pose.position.y;
        //Recalcul map 
        calculGoalPotential();
    }
}

void PlannifNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	ROS_INFO("PlannifNode::mapCallback(msg), msg->taille(w,h) = (%d, %d)\n",
		msg->info.width,
		msg->info.height
	);
    //Initialise toutes les données
    const std::vector<signed char>& data_vector = msg->data;
    map_data = new uint8_t[data_vector.size()];
    std::copy(data_vector.begin(), data_vector.end(), map_data);

    initMaps(msg->info.width, msg->info.height, msg->info.resolution);

    wallDelta = TAILLE_FILTRE_WALL/2;
    preCalculateFilter(wallFilter, wallDelta, TAILLE_FILTRE_WALL, WALL_COEFF_A, WALL_COEFF_B);

    traceDelta = TAILLE_FILTRE_TRACE/2;
    preCalculateFilter(traceFilter, traceDelta, TAILLE_FILTRE_TRACE, TRACE_COEFF_A, TRACE_COEFF_B);
    
    calculInitPotential();

    //On ne reçoit le message qu'une seule fois
    sub_map_.shutdown();
}

void PlannifNode::initMaps(uint32_t width_, uint32_t height_, float resolution_){
	ROS_INFO("PlannifNode::initMaps(width = %d,  height = %d,  resolution_ = %f)\n",
		width_, height_, resolution_
	);
    size_t totalSize = width_ * height_ * sizeof(uint8_t);
		
		initPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    initPotential.layout.dim[0].label = "height";
    initPotential.layout.dim[0].size = height_;
    initPotential.layout.dim[1].label = "width";
    initPotential.layout.dim[1].size = width_;
    initPotential.data.resize(totalSize);
    std::fill(initPotential.data.begin(), initPotential.data.end(), 0);
    

		staticPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    staticPotential.layout.dim[0].label = "height";
    staticPotential.layout.dim[0].size = height_;
    staticPotential.layout.dim[1].label = "width";
    staticPotential.layout.dim[1].size = width_;
    staticPotential.data.resize(totalSize);
    std::fill(staticPotential.data.begin(), initPotential.data.end(), 0);

		tracePotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    tracePotential.layout.dim[0].label = "height";
    tracePotential.layout.dim[0].size = height_;
    tracePotential.layout.dim[1].label = "width";
    tracePotential.layout.dim[1].size = width_;
    tracePotential.data.resize(totalSize);
    std::fill(tracePotential.data.begin(), initPotential.data.end(), 0);

    goalPotential.layout.dim.resize(2, std_msgs::MultiArrayDimension());
    goalPotential.layout.dim[0].label = "height";
    goalPotential.layout.dim[0].size = height_;
    goalPotential.layout.dim[1].label = "width";
    goalPotential.layout.dim[1].size = width_;
    goalPotential.data.resize(totalSize);
    std::fill(goalPotential.data.begin(), goalPotential.data.end(), 0);
}

/**
* @bug Il y a un dépassement de mémoire si taille_filtre est pair.
*/
void PlannifNode::preCalculateFilter(float* filter_, uint8_t delta, int taille_filtre, float coeffA, float coeffB){
	ROS_INFO("PlannifNode::preCalculateFilter(filter, delta=%d, taille_filtre=%d,  coeffA=%f,  coeffB=%f)\n",
		delta, taille_filtre, coeffA, coeffB
	);
    int distance_centre = 0;

    for(int y=-delta; y<delta+1; y++){
        for(int x=-delta; x<delta+1; x++){
            distance_centre = sqrt(x*x + y*y);
            filter_[x+delta + (y+delta)*taille_filtre] = 
                coeffA * exp(-coeffB*distance_centre*distance_centre);
        }
    }
}

/**
* @param signe =-1 ou =1 dépendamment de si on veut additioner ou soustraire le filtre
*/
void PlannifNode::applyFilter(std_msgs::UInt8MultiArray* mapPotential, float* filter_, int indice, uint8_t delta, int coef){
	//ROS_INFO("PlannifNode::applyFilter(mapPotential = %p, filter_, indice=%d,  delta=%d)\n",
	//	mapPotential, indice, delta
	//);
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
	ROS_INFO("PlannifNode::calculInitPotential(), wallDelta=%d\n", wallDelta);
    int indice;
    for(uint32_t y=wallDelta; y<(initPotential.layout.dim[0].size-wallDelta); y++){
        for(uint32_t x=wallDelta; x<(initPotential.layout.dim[1].size-wallDelta); x++){
            indice = x + (y*initPotential.layout.dim[1].size);
            if(map_data[indice] > 0){
                //faire le filtre
                applyFilter(&initPotential, wallFilter, indice, wallDelta, map_data[indice]);
            }
        }
    }
    printMap(initPotential, "initPotential", GOAL_VAL_MAX);
}

void PlannifNode::calculGoalPotential(){
		ROS_INFO("PlannifNode::calculGoalPotential()\n");
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
    
    //Calcul pente
    float pente = (GOAL_VAL_MAX - GOAL_VAL_MIN) / distMax;
    for(uint32_t y=0; y<(goalPotential.layout.dim[0].size); y++){
        for(uint32_t x=0; x<(goalPotential.layout.dim[1].size); x++){
            distance_goal = sqrt((goal_point[0]-x)*(goal_point[0]-x) 
                                        + (goal_point[1]-y)*(goal_point[1]-y));
            goalPotential.data[x + y*goalPotential.layout.dim[1].size] = distance_goal * pente + GOAL_VAL_MIN;
        }
    }
    
    printMap(goalPotential, "goalPotential", GOAL_VAL_MAX);
}

void PlannifNode::calculTracePotential(const ros::TimerEvent& event){
	ROS_INFO("PlannifNode::calculTracePotential(const ros::TimerEvent& event)\n");

    uint32_t indice = pastPosition[0][0] + (pastPosition[0][1]*tracePotential.layout.dim[1].size);
    applyFilter(&tracePotential, traceFilter, indice, traceDelta, -1);
	
		//Met à jour le vecteur pastPositions
    if(pastPosition.size()==TAILLE_MAX_TRACE){
        pastPosition.erase(pastPosition.begin());
    }
    pastPosition.push_back({actualPosition[0], actualPosition[1]});
    
    //tracePotential
    for(uint16_t i=0; i<pastPosition.size(); i++){
        indice = pastPosition[i][0] + (pastPosition[i][1]*tracePotential.layout.dim[1].size);
        applyFilter(&tracePotential, traceFilter, indice, traceDelta, 1);
    }
}   

void PlannifNode::sendStaticPotential(){
	ROS_INFO("PlannifNode::sendStaticPotential()\n");
    //Faire l'addition de toutes les maps 

    //A CHANGER
    pub_staticmap.publish(initPotential);
}

void PlannifNode::printMap(std_msgs::UInt8MultiArray& map, std::string nom, uint16_t valMax) {
	ROS_INFO("PlannifNode::printMap(map, nom)\n");
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
		uint16_t valMax = 255; // trouver la bonne valMax :(
		char tampon[16];
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
	ROS_INFO("PlannifNode::printMap ~%d ko ecrits dans /home/catkin_ws/src/autodrive/mapdump/", width*height/1000);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "plannif_node");
    PlannifNode node;

    ros::spin();
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        //Mettre actions
        node.sendStaticPotential();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

#include "../include/plannif_node.hpp"


PlannifNode::PlannifNode(){
    goal_point[0]=0;
    goal_point[1]=1;

    pub_staticmap = nh_.advertise<nav_msgs::OccupancyGrid>("stc_pot", 1000);
    sub_map_ = nh_.subscribe("/map", 1, &PlannifNode::mapCallback, this);
    sub_goal_ = nh_.subscribe("move_base_simple/goal", 1, &PlannifNode::goalCallback, this);
    sub_odom_ = nh_.subscribe("/odom", 1, &PlannifNode::odomCallback, this);
    ros::Timer timer = nh_.createTimer(ros::Duration(CALCUL_TRACE_MAP_HZ), &PlannifNode::calculTracePotential, this);

}

PlannifNode::~PlannifNode() {
	// Je crois que normalement les vecteurs sont vidés à la destruction des objets
	// mais je fais ça pour m'en assurer.
	initPotential.data.clear();
	initPotential.data.resize(0);
	
	goalPotential.data.clear();
	goalPotential.data.resize(0);
	
	tracePotential.data.clear();
	tracePotential.data.resize(0);
	
	staticPotential.data.clear();
	staticPotential.data.resize(0);
	
	delete[] map_data;
	delete[] wallFilter;
	delete[] traceFilter;
}

void PlannifNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
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
    size_t totalSize = width_ * height_ * sizeof(uint8_t);
		// Inversion par rapport à la doc. Je ne sais pas si c'est grave ??
		// normalement dim[0]<=>"height"
		// et					 dim[1]<=>"width" 
    initPotential.layout.dim[0].label = "width";
    initPotential.layout.dim[0].size = width_;
    initPotential.layout.dim[1].label = "height";
    initPotential.layout.dim[1].size = height_;
    initPotential.data.resize(totalSize);
    std::fill(initPotential.data.begin(), initPotential.data.end(), 0);

    staticPotential.layout.dim[0].label = "width";
    staticPotential.layout.dim[0].size = width_;
    staticPotential.layout.dim[1].label = "height";
    staticPotential.layout.dim[1].size = height_;
    staticPotential.data.resize(totalSize);
    std::fill(staticPotential.data.begin(), initPotential.data.end(), 0);

    tracePotential.layout.dim[0].label = "width";
    tracePotential.layout.dim[0].size = width_;
    tracePotential.layout.dim[1].label = "height";
    tracePotential.layout.dim[1].size = height_;
    tracePotential.data.resize(totalSize);
    std::fill(tracePotential.data.begin(), initPotential.data.end(), 0);
    
    goalPotential.layout.dim[0].label = "width";
    goalPotential.layout.dim[0].size = width_;
    goalPotential.layout.dim[1].label = "height";
    goalPotential.layout.dim[1].size = height_;
    goalPotential.data.resize(totalSize);
    std::fill(goalPotential.data.begin(), goalPotential.data.end(), 0);
}

void PlannifNode::preCalculateFilter(uint8_t* filter_, uint8_t delta, int taille_filtre, float coeffA, float coeffB){
    int distance_centre = 0;
    filter_ = new uint8_t(taille_filtre*taille_filtre);
    
    for(int y=-delta; y<delta+1; y++){
        for(int x=-delta; x<delta+1; x++){
            distance_centre = sqrt(x*x + y*y);
            filter_[x+delta + (y+delta)*taille_filtre] = 
                coeffA * exp(-coeffB*distance_centre*distance_centre);
        }
    }
}

void PlannifNode::applyFilter(std_msgs::UInt8MultiArray* mapPotential, uint8_t* filter_, int indice, uint8_t delta){
    int i = 0;
    int ligne = 0;
    for(int y=-delta; y<delta+1; y++){
        ligne = y*(mapPotential->layout.dim[0].size);
        for(int x=-delta; x<delta+1; x++){
            mapPotential->data[indice + x + ligne] += filter_[i];
            i++;
        }
    }
}

void PlannifNode::calculInitPotential(){
    int indice;
    for(uint32_t y=wallDelta; y<(initPotential.layout.dim[1].size-wallDelta); y++){
        for(uint32_t x=wallDelta; x<(initPotential.layout.dim[0].size-wallDelta); x++){
            indice = x + (y*initPotential.layout.dim[0].size);
            if(map_data[indice] > 0){
                //faire le filtre
                applyFilter(&initPotential, wallFilter, indice, wallDelta);
            }
        }
    }
}

void PlannifNode::calculGoalPotential(){
		ROS_INFO("PlannifNode::calculGoalPotential\n");
    int delta = 0;
    int distance_goal = 0;

    // trouver distMax
    float distMax = 0;
    float dist[4];
    
    dist[0] = sqrt((goal_point[0])*(goal_point[0]) 
                + (goal_point[1])*(goal_point[1]));
		ROS_INFO("PlannifNode::calculGoalPotential -> dist[0]=%f\n", dist[0]);

    dist[1] = sqrt((goal_point[0]-goalPotential.layout.dim[0].size)*(goal_point[0]-goalPotential.layout.dim[0].size) 
                + (goal_point[1])*(goal_point[1]));
		ROS_INFO("PlannifNode::calculGoalPotential -> dist[1]=%f\n", dist[1]);
    dist[2] = sqrt((goal_point[0])*(goal_point[0]) 
                + (goal_point[1]-goalPotential.layout.dim[1].size)*(goal_point[1]-goalPotential.layout.dim[1].size));

    dist[3] = sqrt((goal_point[0]-goalPotential.layout.dim[0].size)*(goal_point[0]-goalPotential.layout.dim[0].size) 
                + (goal_point[1]-goalPotential.layout.dim[1].size)*(goal_point[1]-goalPotential.layout.dim[1].size));

    for(int i=0; i<4; i++){
        if(distMax < dist[i]){
            distMax = dist[i];
        }
    }
    
    //Calcul pente
    float pente = (GOAL_VAL_MAX - GOAL_VAL_MIN) / distMax;
    for(uint32_t y=0; y<(goalPotential.layout.dim[1].size); y++){
        for(uint32_t x=0; x<(goalPotential.layout.dim[0].size); x++){
            distance_goal = sqrt((goal_point[0]-x)*(goal_point[0]-x) 
                                        + (goal_point[1]-y)*(goal_point[1]-y));
            goalPotential.data[x + y*goalPotential.layout.dim[0].size] = distance_goal * pente + GOAL_VAL_MIN;
        }
    }
}

void PlannifNode::calculTracePotential(const ros::TimerEvent& event){
    int indice;
    //tracePotential
    for(int i=0; i<pastPosition.size(); i++){
        indice = pastPosition[i][0] + (pastPosition[i][1]*tracePotential.layout.dim[0].size);
        applyFilter(&tracePotential, traceFilter, indice, traceDelta);
    }

    //Met à jour le vecteur pastPositions
    if(pastPosition.size()==TAILLE_MAX_TRACE){
        pastPosition.erase(pastPosition.begin());
    }
    pastPosition.push_back({actualPosition[0], actualPosition[1]});
}   

void PlannifNode::sendStaticPotential(){
    //Faire l'addition de toutes les maps 

    //A CHANGER
    pub_staticmap.publish(initPotential);
}

void PlannifNode::printMap(std_msgs::UInt8MultiArray& map, std::string nom) {
	// Ouverture du fichier
	//string nomFichier = "carte.pgm";
	std::ofstream wf(nom + ".pgm", std::ios::out | std::ios::binary | std::ios::trunc);
	if (!wf) {
			std::cout << "Cannot open file!" << std::endl;
			return;
	}
	
	// Création de la métadata du pgm
	char metaData[128] = "P5\n";
	
	size_t metaDataSize = 3;

	uint32_t width = map.layout.dim[1].size;
	uint32_t height = map.layout.dim[0].size;
	if (width && height) {
		uint16_t valMax = 100; // trouver la bonne valMax :(
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

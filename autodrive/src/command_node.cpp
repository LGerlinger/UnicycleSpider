# include "../include/command_node.hpp"

CommandNode::CommandNode(): tfBuffer(), tfListener(tfBuffer) {
	// map = (uint8_t*)malloc(1);
	width = 1;
	height = 1;


	for (uint8_t y=0; y<TAILLE_FILTRE; y++) {
		for (uint8_t x=0; x<TAILLE_FILTRE; x++) {
			filtre[y][x] = 1/(sqrt(
				(TAILLE_FILTRE/2 - x) * (TAILLE_FILTRE/2 - x) +
				(TAILLE_FILTRE/2 - y) * (TAILLE_FILTRE/2 - y)
			)+1);
		}	
	}

	//Publish et Subscriber
	pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	sub_map_ = nh_.subscribe("/gmap", 1, &CommandNode::mapCallback, this);
	sub_pot_map_ = nh_.subscribe("stc_pot", 1000, &CommandNode::getPotMapCallback, this);
	sub_activation_ = nh_.subscribe("rbt_actv", 1, &CommandNode::changeState, this);

}

CommandNode::~CommandNode() {
	if (map != NULL) free(map);
}

//NOTE POUR PLANNIF NODE : FAIRE ATTENTION AUX TIMER LANCEE ALORS QUE LE STOP EST ACTIF !!!
void CommandNode::changeState(const std_msgs::Bool::ConstPtr& stop){
	stop_ = stop->data;
	if(stop_){
		ROS_INFO("COMMANDE : STOP TOUT");
		geometry_msgs::Twist msg;
		msg.angular.z = 0;
		msg.linear.x = 0;
		pub_cmd_vel_.publish(msg);
		if(!first_init){
			timer.stop();
			first_init = true;
		}
    }else if(first_init){
		timer_start = nh_.createTimer(ros::Duration(0.5f), &CommandNode::checkMapInit, this);
	}
}

void CommandNode::checkMapInit(const ros::TimerEvent& event){
	if(!stop_){
		if(!std::isnan(originMap.position.x) && map != nullptr){
			timer = nh_.createTimer(ros::Duration(1.f/SEND_CMD_HZ), &CommandNode::Map2Command, this);

			timer_start.stop();
			first_init = false;
		}
	}else{
		timer_start.stop();
	}
}


void CommandNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	//ROS_INFO("CommandNode::mapCallback,  Position mesurée [%d, %d, %f]", pixelPosition[0], pixelPosition[1], posture[2]);
	resolution = msg->info.resolution;
	originMap = msg->info.origin;
}

void CommandNode::getPotMapCallback(const std_msgs::UInt8MultiArray& msg) {
	if (height != msg.layout.dim[0].size || width != msg.layout.dim[1].size) {
		// free(map);
		height = msg.layout.dim[0].size;
		width = msg.layout.dim[1].size;
		map = (uint8_t*)malloc((size_t) (width * height));
		if (map == NULL) {
			ROS_INFO("CommandNode::getMapCallback -> échec de l'allocation mémoire\n");
			ROS_INFO("CommandNode::getMapCallback -> height : %d,  width : %d,  mémoire demandée : %d\n", height, width, height*width);
		}
	}
	std::copy(msg.data.begin(), msg.data.end(), map);
	
	//ROS_INFO("CommandNode::getPotMapCallback, height : %d,  width : %d\n", height, width);
}

/**
 * @brief Récupére la position du robot et calcul sa direction
*/
void CommandNode::Map2Command(const ros::TimerEvent& event) {
	//ROS_INFO("CommandNode::Map2Command deb");
	//ROS_INFO("CommandNode::Map2Command passe try");

	float nouveauGradient[2] = {0,0};
	int64_t pXd, pYd;
	int8_t delta = TAILLE_FILTRE/2;
	//posture[2] = - 2* M_PI/3;

	//Récupération de la position actuelle
	getRobotPos();
	
	//Calcul du gradient
	for (int8_t dy = -delta; dy < delta+1; dy++) {
		pYd = pixelPosition[1] + dy;
		if (0 <= pYd && pYd < height) {
			for (int8_t dx = -delta; dx < delta+1; dx++) {
				pXd = pixelPosition[0] + dx;
				if (0 <= pXd && pXd < width) {
					// Trouver le sens
					nouveauGradient[0] -= dx * filtre[dy+delta][dx+delta] * map[pYd*width + pXd];
					nouveauGradient[1] -= dy * filtre[dy+delta][dx+delta] * map[pYd*width + pXd]; 
				}
			}
		}
	}
	gradient[0] = coefMomentum * gradient[0] + nouveauGradient[0];
	gradient[1] = coefMomentum * gradient[1] + nouveauGradient[1];

	// ROS_INFO("CommandNode::Map2Command() -- grad=(%f, %f)", gradient[0], gradient[1]);
	
	float norme = sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1]);

	// Transformer le sens en commande vitesse linéaire + angulaire
	// on veut beaucoup tourner -> vitesse linéaire basse
	// pente de potentiel faible -> vitesse linéaire basse
	geometry_msgs::Twist msg;
	
	if (norme < 0.01f) {
		msg.angular.z = 0.5f;
		msg.linear.x = 0.f;
	}
	else {
		float angleVoulu = - atan2(-gradient[1], gradient[0]);
		float diffAngle = angleVoulu - posture[2];
		
		if (diffAngle > M_PI) diffAngle = -2*M_PI + diffAngle;
		else if (diffAngle < -M_PI) diffAngle = 2*M_PI + diffAngle;
		
		msg.angular.z = - coefCmdRot * diffAngle;
		msg.linear.x = coefCmdLin * (M_PI - abs(diffAngle))/M_PI * norme;
	}
	
	pub_cmd_vel_.publish(msg);
}

void CommandNode::getRobotPos(){
	try {
		tfRobot2Map = tfBuffer.lookupTransform("base_link", "map", ros::Time(0));
		pixelPosition[0] = (tfRobot2Map.transform.translation.x - originMap.position.x)/resolution;
		pixelPosition[1] = height - (tfRobot2Map.transform.translation.y - originMap.position.y)/resolution;
		
		geometry_msgs::Quaternion& q = tfRobot2Map.transform.rotation;
		posture[2] = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
		
	}catch (tf2::TransformException& ex) {
			ROS_WARN("%s", ex.what());
	}
}

/**
 * @brief Enregistrement d'une map
*/
void CommandNode::printMap(const std_msgs::UInt8MultiArray& carte, std::string nom, uint16_t valMax) {
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

	uint32_t width = carte.layout.dim[1].size;
	uint32_t height = carte.layout.dim[0].size;
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
		wf.write((char*)carte.data.data(), (size_t)(width*height));
	}
	// ROS_INFO("PlannifNode::printMap ~%d ko ecrits dans /home/catkin_ws/src/autodrive/mapdump/", width*height/1000);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_node");
	CommandNode cmdNode;
	
	ros::spin();
	return 0;
}

// %EndTag(FULLTEXT)%

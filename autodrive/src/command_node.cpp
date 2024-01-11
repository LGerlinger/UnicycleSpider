# include "../include/command_node.hpp"

CommandNode::CommandNode() {
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	map_sub = n.subscribe("stat_pot", 1000, &CommandNode::getMapCallback, this);
	odom_sub = n.subscribe("odom", 1000, &CommandNode::getOdomCallback, this);
	map = (uint8_t*)malloc(1);
	width = 1;
	height = 1;
}

void CommandNode::Map2Command() {
	// pose est sensée être dans le tableau
	float nouveauGradient[2] = {0,0};
	uint32_t posX = round(posture[0]);
	uint32_t posY = round(posture[1]);
	int64_t pXd, pYd;
	int8_t delta = tailleFiltre/2;
	
	for (int8_t dy = -delta; dy < delta+1; dy++) {
		pYd = posY + dy;
		if (0 <= pYd && pYd < height) {
			for (int8_t dx = delta; dx < delta+1; dx++) {
				pXd = posX + dx;
				if (0 <= pXd && pXd < width) {
					// Trouver le sens
					nouveauGradient[0] += dx * filtre[dy+delta][dx+delta] * map[pYd*width + pXd];
					nouveauGradient[1] += dy * filtre[dy+delta][dx+delta] * map[pYd*width + pXd]; 
				}
			}
		}
	}
	nouveauGradient[0] *= -1;
	nouveauGradient[0] += coefMomentum * gradient[0];
	
	nouveauGradient[1] *= -1;
	nouveauGradient[1] += coefMomentum * gradient[1];
	float norme = sqrt(nouveauGradient[0] * nouveauGradient[0] + nouveauGradient[1] * nouveauGradient[1]);

	// Transformer le sens en commande vitesse linéaire + angulaire
	// on veut beaucoup tourner -> vitesse linéaire basse
	// pente de potentiel faible -> vitesse linéaire basse
	geometry_msgs::Twist msg;

	float diffAngle = posture[2] - atan2(nouveauGradient[1], nouveauGradient[0]);
	if (diffAngle > M_PI) diffAngle = -2*M_PI + diffAngle;
	if (diffAngle < -M_PI) diffAngle = 2*M_PI + diffAngle;
	msg.angular.z = diffAngle/2;
	
	msg.linear.x = coefCommande * abs(diffAngle)/M_PI * norme;
	//
	// ROS_INFO("%s", msg.data.c_str());

	cmd_vel_pub.publish(msg);
}

void CommandNode::getMapCallback(const std_msgs::UInt8MultiArray& msg) {
	if (height != msg.layout.dim[0].size || width != msg.layout.dim[1].size) {
		free(map);
	}
	width = msg.layout.dim[1].size;
	height = msg.layout.dim[0].size;
	map = (uint8_t*)malloc((size_t) (width * height));
	memcpy(map, &msg.data, (size_t) (width * height));
}

void CommandNode::getOdomCallback(const nav_msgs::Odometry& msg) {
	posture[0] = msg.pose.pose.position.x;
	posture[1] = msg.pose.pose.position.y;
	auto orientation = &(msg.pose.pose.orientation); // ça marche ?? Je sais pas :/
	posture[2] = atan2(2 * (orientation->w * orientation->z + orientation->x * orientation->y), 1 - 2 * (orientation->y*orientation->y + orientation->z*orientation->z));
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_node");
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	
	return 0;
}

// %EndTag(FULLTEXT)%

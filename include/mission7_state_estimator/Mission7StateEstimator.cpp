/*
 * Mission7StateEstimator.cpp
 *
 *  Created on: May 12, 2018
 *      Author: kevin
 */

#include "Mission7StateEstimator.h"

Mission7StateEstimator::Mission7StateEstimator() {

	ros::NodeHandle nh;

	//setup publishers
	this->quad_odom_publisher = nh.advertise<nav_msgs::Odometry>("state_estimator/odom", 10);

}

Mission7StateEstimator::~Mission7StateEstimator() {

}


void Mission7StateEstimator::main_loop(){

	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		ros::spinOnce();

		//update using measurements
		this->updateState();	

		loop_rate.sleep();
	}
}

void Mission7StateEstimator::publishQuadOdometry(){
		nav_msgs::Odometry odom;
}

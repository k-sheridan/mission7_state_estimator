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

	//setup subs
	ros::Subscriber odom_sub = nh.subscribe("stereo_odometer/odom", 10, &Mission7StateEstimator::odomCallback, this);
	ros::Subscriber imu_sub = nh.subscribe("mavros/imu/data", 10, &Mission7StateEstimator::imuCallback, this);
	ros::Subscriber range_sub = nh.subscribe("mavros/distance_sensor/lidar", 10, &Mission7StateEstimator::rangeCallback, this);

	//start est loop
	this->main_loop();
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

/*
 * Mission7StateEstimator.cpp
 *
 *  Created on: May 12, 2018
 *      Author: kevin
 */

#include "Mission7StateEstimator.h"

Mission7StateEstimator::Mission7StateEstimator() {
	// TODO Auto-generated constructor stub

}

Mission7StateEstimator::~Mission7StateEstimator() {
	// TODO Auto-generated destructor stub
}


void Mission7StateEstimator::main_loop(){

	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		ros::spinOnce();

		//TODO update using measurements

		loop_rate.sleep();
	}
}

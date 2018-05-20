/*
 * Update.cpp
 *
 *  Created on: May 20, 2018
 *      Author: kevin
 */

#include "Mission7StateEstimator.h"


bool timeSort(GenericMeasurement i, GenericMeasurement j){i.getTime() < j.getTime();}

/*
 * sorts and applies measurements by their time stamp
 */
void Mission7StateEstimator::updateState(){

	//sort
	std::sort(this->measurements.begin(), this->measurements.end(), timeSort);

	// prune old measurements
	for(std::deque<GenericMeasurement>::iterator it = this->measurements.begin(); it != this->measurements.end();){
		if(it->getTime() < this->quad_state.getTime()){
			it = this->measurements.erase(it);
		}else{
			it++;
		}
	}

	// apply the remaining updates in order


}


void Mission7StateEstimator::IMUUpdate(sensor_msgs::Imu){
//TODO
}
void Mission7StateEstimator::odomUpdate(nav_msgs::Odometry){
//TODO
}
void Mission7StateEstimator::rangeUpdate(sensor_msgs::Range){
//TODO
}

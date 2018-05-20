/*
 * Callback.cpp
 *
 *  Created on: May 20, 2018
 *      Author: kevin
 */

#include "Mission7StateEstimator.h"

void Mission7StateEstimator::odomCallback(const nav_msgs::OdometryConstPtr& msg){
	this->measurements.push_back(GenericMeasurement(*msg));
}
void Mission7StateEstimator::imuCallback(const sensor_msgs::ImuConstPtr& msg){
	this->measurements.push_back(GenericMeasurement(*msg));
}
void Mission7StateEstimator::rangeCallback(const sensor_msgs::RangeConstPtr& msg){
	this->measurements.push_back(GenericMeasurement(*msg));
}


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

	// apply updates
	for(std::deque<GenericMeasurement>::iterator it = this->measurements.begin(); it != this->measurements.end();){
		//check if measurement is too old
		double dt = (it->getTime() - this->quad_state.getTime()).toSec();
		if(-dt <= MAXIMUM_MEASUREMENT_AGE){
			// process if necessary
			if(dt > 0){
				this->quad_state = this->processQuad(this->quad_state, dt);
			}
			
			//TODO check for outliers
			
			//update
			this->update(*it);	
			
		}


		this->measurements.pop_front();
	}


}

void Mission7StateEstimator::update(GenericMeasurement z){
	switch(z.type){
		case GenericMeasurement::IMU:
		IMUUpdate(z.imu_msg);
		break;

		case GenericMeasurement::ODOM:
		odomUpdate(z.odom_msg);
		break;

		case GenericMeasurement::RANGE:
		rangeUpdate(z.range_msg);
		break;
	}
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

Eigen::Matrix<double, 6, 1> Mission7StateEstimator::IMUMeasurement(QuadState x){
	Eigen::Matrix<double, 6, 1> inertial;

	Eigen::Matrix<double, 3, 1> accel;
	accel << 0, 0, GRAVITY;

	accel = Sophus::SO3::exp(-x.getAngularTwist()) * accel + x.getAcceleration() + x.getAccelBiases();

	inertial << accel, x.getOmega();
}

// bdx, bdy, bdz, bwx, bwy, bwz
Eigen::Matrix<double, 6, 1> Mission7StateEstimator::odomMeasurement(QuadState x){
	Eigen::Matrix<double, 6, 1> twist;
	twist << x.getVelocity(), x.getOmega();

	return twist;
}
double Mission7StateEstimator::rangeMeasurement(QuadState x, tf::Transform b2l){

}


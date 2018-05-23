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
	// compute numerical jacobian
	Eigen::Matrix<double, 6, QUAD_STATE_SIZE> H;
	
	for(int i = 0; i < QUAD_STATE_SIZE; i++){
		Eigen::Matrix<double, QUAD_STATE_SIZE, 1> mean = this->quad_state.getMean();
		mean(i) += JACOBIAN_DELTA;
		
		QuadState test;
		test.setMean(mean);

		Eigen::Matrix<double, 6, 1> high = this->IMUMeasurement(test);

		mean(i) -= 2*JACOBIAN_DELTA;

		test.setMean(mean);

		Eigen::Matrix<double, 6, 1> low = this->IMUMeasurement(test);

		H.block(6, 1, 0, i) = (1 / (2*JACOBIAN_DELTA))*(high - low);
	}

	// kalman update
}
void Mission7StateEstimator::odomUpdate(nav_msgs::Odometry){
	// compute numerical jacobian
	Eigen::Matrix<double, 6, QUAD_STATE_SIZE> H;
	
	for(int i = 0; i < QUAD_STATE_SIZE; i++){
		Eigen::Matrix<double, QUAD_STATE_SIZE, 1> mean = this->quad_state.getMean();
		mean(i) += JACOBIAN_DELTA;
		
		QuadState test;
		test.setMean(mean);

		Eigen::Matrix<double, 6, 1> high = this->odomMeasurement(test);

		mean(i) -= 2*JACOBIAN_DELTA;

		test.setMean(mean);

		Eigen::Matrix<double, 6, 1> low = this->odomMeasurement(test);

		H.block(6, 1, 0, i) = (1 / (2*JACOBIAN_DELTA))*(high - low);
	}

	// kalman update
}
void Mission7StateEstimator::rangeUpdate(sensor_msgs::Range){
	tf::StampedTransform transform;
	try{
		tf_listener.lookupTransform(BASE_FRAME, LIDAR_FRAME,  
                               ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_WARN("%s",ex.what());
		ROS_WARN("skipping");
		return;
	}
	
	// compute numerical jacobian
	Eigen::Matrix<double, 1, QUAD_STATE_SIZE> H;
	
	for(int i = 0; i < QUAD_STATE_SIZE; i++){
		Eigen::Matrix<double, QUAD_STATE_SIZE, 1> mean = this->quad_state.getMean();
		mean(i) += JACOBIAN_DELTA;
		
		QuadState test;
		test.setMean(mean);

		double high = this->rangeMeasurement(test, transform);

		mean(i) -= 2*JACOBIAN_DELTA;

		test.setMean(mean);

		double low = this->rangeMeasurement(test, transform);

		H(0, i) = (high - low) / (2*JACOBIAN_DELTA);
	}

	// kalman update
	
}

Eigen::Matrix<double, 6, 1> Mission7StateEstimator::IMUMeasurement(QuadState x){
	Eigen::Matrix<double, 6, 1> inertial;

	Eigen::Matrix<double, 3, 1> accel;
	accel << 0, 0, GRAVITY;

	accel = Sophus::SO3::exp(-x.getAngle()) * accel + x.getAcceleration() + x.getAccelBiases();

	inertial << accel, x.getOmega();
}

// bdx, bdy, bdz, bwx, bwy, bwz
Eigen::Matrix<double, 6, 1> Mission7StateEstimator::odomMeasurement(QuadState x){
	Eigen::Matrix<double, 6, 1> twist;
	twist << x.getVelocity(), x.getOmega();

	return twist;
}

//assumes lidar is down facing
double Mission7StateEstimator::rangeMeasurement(QuadState x, tf::Transform b2l){
	Eigen::Matrix<double, 3, 1> r_lidar(b2l.getOrigin().x(), b2l.getOrigin().y(), b2l.getOrigin().z());

	Sophus::SO3d rot = Sophus::SO3d::exp(x.getAngle());

	Eigen::Matrix<double, 3, 1> pos = x.getPosition() + rot * r_lidar;

	Eigen::Matrix<double, 3, 1> bearing;
	bearing << 0, 0, 1;

	bearing = rot * bearing;

	double range = pos(2) / bearing(2);

	return range;
}


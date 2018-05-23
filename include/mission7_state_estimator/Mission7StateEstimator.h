/*
 * Mission7StateEstimator.h
 *
 *  Created on: May 12, 2018
 *      Author: kevin
 */

#ifndef MISSION7_STATE_ESTIMATOR_INCLUDE_MISSION7_STATE_ESTIMATOR_MISSION7STATEESTIMATOR_H_
#define MISSION7_STATE_ESTIMATOR_INCLUDE_MISSION7_STATE_ESTIMATOR_MISSION7STATEESTIMATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <list>
#include <deque>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "Params.h"
#include "Types.h"

class Mission7StateEstimator {
public:

	GroundRobotState ground_robot_state;
	QuadState quad_state;

	ros::Publisher quad_odom_publisher;

	tf::TransformListener tf_listener;

	std::deque<GenericMeasurement> measurements; // deque of all measurements


	Mission7StateEstimator();
	virtual ~Mission7StateEstimator();

	void odomCallback(const nav_msgs::OdometryConstPtr& msg);
	void imuCallback(const sensor_msgs::ImuConstPtr& msg);
	void rangeCallback(const sensor_msgs::RangeConstPtr& msg);

	void updateState();

	void update(GenericMeasurement z);

	void IMUUpdate(sensor_msgs::Imu);
	void odomUpdate(nav_msgs::Odometry);
	void rangeUpdate(sensor_msgs::Range);

	Eigen::Matrix<double, 6, 1> IMUMeasurement(QuadState x);
	Eigen::Matrix<double, 6, 1> odomMeasurement(QuadState x);
	double rangeMeasurement(QuadState x, tf::Transform b2l);

	void publishQuadOdometry();

	GroundRobotState processGroundRobots(GroundRobotState prior, double dt);

	GroundRobotState processGroundRobotsAndPropagateUncertainty(GroundRobotState prior, double dt);

	QuadState processQuad(QuadState prior, double dt);

	QuadState processQuadAndPropagateUncertainty(QuadState prior, double dt);

	void main_loop();
};

#endif /* MISSION7_STATE_ESTIMATOR_INCLUDE_MISSION7_STATE_ESTIMATOR_MISSION7STATEESTIMATOR_H_ */

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
#include <deque>

#include <tf/transform_listener.h>

#include "Types.h"

class Mission7StateEstimator {
public:

	GroundRobotState ground_robot_state;
	QuadState quad_state;

	tf::TransformListener tf_listener;

	std::deque<sensor_msgs::Imu> imu_buffer;
	std::deque<nav_msgs::Odometry> odom_buffer;
	std::deque<sensor_msgs::Range> range_buffer;


	Mission7StateEstimator();
	virtual ~Mission7StateEstimator();


	GroundRobotState processGroundRobots(GroundRobotState prior, double dt);

	GroundRobotState processGroundRobotsAndPropagateUncertainty(GroundRobotState prior, double dt);

	QuadState processQuad(QuadState prior, double dt);

	QuadState processQuadAndPropagateUncertainty(QuadState prior, double dt);
};

#endif /* MISSION7_STATE_ESTIMATOR_INCLUDE_MISSION7_STATE_ESTIMATOR_MISSION7STATEESTIMATOR_H_ */

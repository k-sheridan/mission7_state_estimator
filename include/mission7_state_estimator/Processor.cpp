/*
 * Processor.cpp
 *
 *  Created on: May 12, 2018
 *      Author: kevin
 */

#include "Mission7StateEstimator.h"

QuadState Mission7StateEstimator::processQuad(QuadState prior, double dt){
	QuadState post = prior;

	Eigen::Matrix<double, 6, 1> twist;
	twist << prior.getVelocity() * dt + prior.getAcceleration() * 0.5 * dt * dt, prior.getOmega() * dt;

	post.setLinearTwist(twist.block(0, 0, 3, 1));
	post.setAngularTwist(twist.block(3, 0, 3, 1));

	post.setPose(prior.getPose() * Sophus::SE3d::exp(twist));

	post.setVelocity(prior.getVelocity() + prior.getAcceleration() * dt);
}

QuadState Mission7StateEstimator::processQuadAndPropagateUncertainty(QuadState prior, double dt){

}

GroundRobotState Mission7StateEstimator::processGroundRobots(GroundRobotState prior, double dt){

}

GroundRobotState Mission7StateEstimator::processGroundRobotsAndPropagateUncertainty(GroundRobotState prior, double dt){

}



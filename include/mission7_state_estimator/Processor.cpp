/*
 * Processor.cpp
 *
 *  Created on: May 12, 2018
 *      Author: kevin
 */

#include "Mission7StateEstimator.h"

QuadState Mission7StateEstimator::processQuad(QuadState prior, double dt){
	QuadState post = prior;

	post.setPosition(prior.getAttitude() * (prior.getVelocity() * dt + 0.5 * prior.getAcceleration() * dt * dt));

	post.setAttitude(prior.getAttitude() * Sophus::SO3::exp(prior.getOmega() * dt));

	post.setAngle(post.getAttitude().log());

	post.setVelocity(prior.getVelocity() + prior.getAcceleration() * dt);
}

QuadState Mission7StateEstimator::processQuadAndPropagateUncertainty(QuadState prior, double dt){

}

GroundRobotState Mission7StateEstimator::processGroundRobots(GroundRobotState prior, double dt){

}

GroundRobotState Mission7StateEstimator::processGroundRobotsAndPropagateUncertainty(GroundRobotState prior, double dt){

}



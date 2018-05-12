/*
 * Mission7StateEstimator.h
 *
 *  Created on: May 12, 2018
 *      Author: kevin
 */

#ifndef MISSION7_STATE_ESTIMATOR_INCLUDE_MISSION7_STATE_ESTIMATOR_MISSION7STATEESTIMATOR_H_
#define MISSION7_STATE_ESTIMATOR_INCLUDE_MISSION7_STATE_ESTIMATOR_MISSION7STATEESTIMATOR_H_

#include "Types.h"

class Mission7StateEstimator {
public:
	Mission7StateEstimator();
	virtual ~Mission7StateEstimator();


	Mission7State process(Mission7State state, double dt);

	Mission7State processAndPropagateUncertainty(Mission7State state, double dt);
};

#endif /* MISSION7_STATE_ESTIMATOR_INCLUDE_MISSION7_STATE_ESTIMATOR_MISSION7STATEESTIMATOR_H_ */

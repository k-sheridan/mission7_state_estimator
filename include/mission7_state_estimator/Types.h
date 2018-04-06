#ifndef MISSION7_STATE_ESTIMATOR_MISSION7_STATE_ESTIMATOR_TYPES_H_
#define MISSION7_STATE_ESTIMATOR_MISSION7_STATE_ESTIMATOR_TYPES_H_

#include <Eigen/core>

//NOTE: the standard floating point precision we are using is double


// x, y, z, thx, thy, thz, dx, dy, dz, wx, wy, wz, ax, ay, az
#define QUAD_STATE_SIZE 15
// x, y, theta
#define OBSTACLE_STATE_SIZE 3
// x, y, theta, timer
#define TARGET_STATE_SIZE 4

struct TargetState{
  Eigen::Matrix<double, TARGET_STATE_SIZE, 1> mean;
}

struct ObstacleState{
  Eigen::Matrix<double, OBSTACLE_STATE_SIZE, 1> mean;
}

struct QuadState{
  Eigen::Matrix<double, QUAD_STATE_SIZE, 1> mean;
}

struct Mission7State{
private:
  QuadState quad_state;
  std::vector<ObstacleState> obstacle_states;
  std::vector<TargetState> target_states;

  Eigen::MatrixXd covariance;

public:

/*
* ORDER: QUAD, OBSTACLE, TARGETS
*/
  Eigen::Matrix<double, Eigen::Dynamic, 1> toStateVector(){

  }

  void fromStateVector(Eigen::Matrix<double, Eigen::Dynamic, 1> vec){
    // make sure that you don't add extra variables somehow
    ROS_ASSERT(vec.rows() == QUAD_STATE_SIZE + obstacle_states.size()*OBSTACLE_STATE_SIZE + target_states.size()*TARGET_STATE_SIZE);


  }


};

#endif

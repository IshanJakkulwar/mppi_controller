// Explained

#ifndef MPPI_CONTROLLER__MODELS__DRONE_STATE_HPP_ //ensures no renaming issues
#define MPPI_CONTROLLER__MODELS__DRONE_STATE_HPP_

#include <Eigen/Dense>// the best library to replace std::vector, can run things SIMD(single instruction multiple data)

namespace mppi_controller //create namespace(good for ensuring less errors from different folders/packages clashing)
{

struct DroneState//public version of a class, good for storing valuable, universal data
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};  // w, x, y, z, 4d vector where one part(w)is real, the other 3 are imaginary that encode a rotation axis and angle without gimbal lock 
  Eigen::Vector3d angular_velocity{0.0, 0.0, 0.0};
  double timestamp{0.0};  // seconds
};

}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__MODELS__DRONE_STATE_HPP_
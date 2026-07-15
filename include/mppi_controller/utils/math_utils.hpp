#ifndef MPPI_CONTROLLER__UTILS__MATH_UTILS_HPP_
#define MPPI_CONTROLLER__UTILS__MATH_UTILS_HPP_

#include <algorithm>
#include <random>
#include <Eigen/Dense>

namespace mppi_controller
{
namespace math_utils
{

template<typename T>//scalar clamp
inline T clamp(T value, T min_val, T max_val)
{
  return std::max(min_val, std::min(value, max_val));//evaluates inner paramters first. Finds whether the value is smaller than the max value, and larger than the minimum value
}

inline Eigen::Vector3d clampVector(// this is a vector scaling/clamp method. Say you wanted to go diagonally at 100m/s (north and east) but was teuncated to 60m/s, it may go more north than east, this takes care of that
  const Eigen::Vector3d & v, double max_norm)
{
  double norm = v.norm();// uses pythagorean theorem to find the length of arrow
  if (norm > max_norm && norm > 1e-9) {//checks whether the norm calculated is greater than max_norm(which is set separelty) and if it is non zero(which is the point of the 1e--9)
    return v * (max_norm / norm); // if so, it multiplies v by the scale between the two, so say the norm was 5 and max norm was 2, it would be multiplying the vector by 2/5
  }
  return v;
}

// Thin wrapper around a Mersenne Twister for sampling per-axis Gaussian
// noise. One instance should be owned by whatever controller needs
// repeated sampling (e.g. MPPI), not constructed fresh every call --
// std::mt19937 seeding is relatively expensive.
class GaussianSampler// this is because computers are incapable of being completely random. Therefore, it fakes being random through an algorithm
{
public://random device uses the random fluctuations in battery health etc. in order to create a truly random number
  explicit GaussianSampler(unsigned int seed = std::random_device{}())//explicit is very useful as it stops C++ from being too helpful and creating its own gaussian sampler which can take much more cpu and compute and therefore ruin the whole premise 
  : generator_(seed), dist_(0.0, 1.0)//0.0 indicates it should be close to 0, 1.0 is the spread 
  {
  }

  // Returns a 3D sample with independent Gaussian noise per axis,
  // each scaled by the corresponding entry in std_dev.
  Eigen::Vector3d sample(const Eigen::Vector3d & std_dev)
  {
    return Eigen::Vector3d(
      dist_(generator_) * std_dev.x(),//manipulating with the random number
      dist_(generator_) * std_dev.y(),
      dist_(generator_) * std_dev.z());
  }

private://stops other files changinng it, can access through function sample
  std::mt19937 generator_;
  std::normal_distribution<double> dist_;
};

}  // namespace math_utils
}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__UTILS__MATH_UTILS_HPP_
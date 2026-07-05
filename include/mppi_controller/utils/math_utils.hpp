#ifndef MPPI_CONTROLLER__UTILS__MATH_UTILS_HPP_
#define MPPI_CONTROLLER__UTILS__MATH_UTILS_HPP_

#include <algorithm>
#include <random>
#include <Eigen/Dense>

namespace mppi_controller
{
namespace math_utils
{

template<typename T>
inline T clamp(T value, T min_val, T max_val)
{
  return std::max(min_val, std::min(value, max_val));
}

inline Eigen::Vector3d clampVector(
  const Eigen::Vector3d & v, double max_norm)
{
  double norm = v.norm();
  if (norm > max_norm && norm > 1e-9) {
    return v * (max_norm / norm);
  }
  return v;
}

// Thin wrapper around a Mersenne Twister for sampling per-axis Gaussian
// noise. One instance should be owned by whatever controller needs
// repeated sampling (e.g. MPPI), not constructed fresh every call --
// std::mt19937 seeding is relatively expensive.
class GaussianSampler
{
public:
  explicit GaussianSampler(unsigned int seed = std::random_device{}())
  : generator_(seed), dist_(0.0, 1.0)
  {
  }

  // Returns a 3D sample with independent Gaussian noise per axis,
  // each scaled by the corresponding entry in std_dev.
  Eigen::Vector3d sample(const Eigen::Vector3d & std_dev)
  {
    return Eigen::Vector3d(
      dist_(generator_) * std_dev.x(),
      dist_(generator_) * std_dev.y(),
      dist_(generator_) * std_dev.z());
  }

private:
  std::mt19937 generator_;
  std::normal_distribution<double> dist_;
};

}  // namespace math_utils
}  // namespace mppi_controller

#endif  // MPPI_CONTROLLER__UTILS__MATH_UTILS_HPP_
#include <chrono>
#include <memory>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "mppi_controller/models/drone_state.hpp"
#include "mppi_controller/controllers/mppi_controller.hpp"
#include "mppi_controller/controllers/anytime_scheduler.hpp"
#include "mppi_controller/utils/math_utils.hpp"

using namespace std::chrono_literals;
using mppi_controller::DroneState;
using mppi_controller::MppiController;
using mppi_controller::MppiConfig;
using mppi_controller::AnytimeScheduler;
using mppi_controller::SchedulerConfig;

class OffboardNode : public rclcpp::Node
{
public:
  OffboardNode()
  : Node("offboard_node"),
    current_wp_index_(0),
    mppi_(makeMppiConfig()),
    scheduler_(makeSchedulerConfig()),
    next_n_(200)
  {
    waypoints_ = {
      {0.0, 0.0, 5.0},
      {2.0, 0.0, 5.0},
      {2.0, 2.0, 5.0},
      {0.0, 2.0, 5.0},
      {0.0, 0.0, 5.0}
    };

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::SensorDataQoS(),
      std::bind(&OffboardNode::stateCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&OffboardNode::poseCallback, this, std::placeholders::_1));

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 10);

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
      "/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
      "/mavros/set_mode");

    last_loop_time_ = this->now();
    last_request_time_ = this->now();

    timer_ = this->create_wall_timer(
      50ms, std::bind(&OffboardNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "offboard_node (Anytime MPPI) started");
  }

private:
  static MppiConfig makeMppiConfig()
  {
    MppiConfig config;
    config.horizon = 20;
    config.num_samples = 200;  // used only as the very first call's N
    config.dt = 0.1;
    config.state_weight = Eigen::Vector3d(10.0, 10.0, 10.0);
    config.control_weight = Eigen::Vector3d(0.3, 0.3, 0.3);   // tuned
    config.terminal_weight = Eigen::Vector3d(20.0, 20.0, 20.0);
    config.noise_std = Eigen::Vector3d(0.5, 0.5, 0.5);        // tuned
    config.max_speed = 2.0;
    config.lambda = 3.0;                                       // tuned
    return config;
  }

  static SchedulerConfig makeSchedulerConfig()
  {
    SchedulerConfig config;
    config.n_min = 20;
    config.n_max = 400;
    config.n_default = 200;
    config.target_loop_time = 0.05;   // matches the 20Hz control timer
    config.deadline_margin = 0.8;     // budget 40ms of the 50ms cycle for MPPI
    config.smoothing_window = 3;
    return config;
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    drone_state_.position = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    drone_state_.orientation = Eigen::Quaterniond(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);
    drone_state_.timestamp = this->now().seconds();

    if (!have_pose_) {
      ground_position_ = drone_state_.position;
      takeoff_start_time_ = this->now();
    }
    have_pose_ = true;
  }

  bool reachedCurrentWaypoint()
  {
    if (!have_pose_) {
      return false;
    }
    Eigen::Vector3d target(
      waypoints_[current_wp_index_][0],
      waypoints_[current_wp_index_][1],
      waypoints_[current_wp_index_][2]);
    return (drone_state_.position - target).norm() < position_tolerance_;
  }

  Eigen::Vector3d computeTarget()
  {
    if (taking_off_) {
      double elapsed = (this->now() - takeoff_start_time_).seconds();
      double t = mppi_controller::math_utils::clamp(
        elapsed / takeoff_duration_, 0.0, 1.0);

      Eigen::Vector3d target(
        ground_position_.x(),
        ground_position_.y(),
        ground_position_.z() + t * (hover_altitude_ - ground_position_.z()));

      if (t >= 1.0) {
        taking_off_ = false;
        mppi_.reset();
        scheduler_.reset();
        next_n_ = 200;
        RCLCPP_INFO(this->get_logger(), "Takeoff ramp complete, switching to Anytime MPPI waypoint tracking");
      }
      return target;
    }

    return Eigen::Vector3d(
      waypoints_[current_wp_index_][0],
      waypoints_[current_wp_index_][1],
      waypoints_[current_wp_index_][2]);
  }

  void publishVelocity(const Eigen::Vector3d & vel)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.twist.linear.x = vel.x();
    msg.twist.linear.y = vel.y();
    msg.twist.linear.z = vel.z();
    velocity_pub_->publish(msg);
  }

  void timerCallback()
  {
    rclcpp::Time now = this->now();
    double dt = (now - last_loop_time_).seconds();
    last_loop_time_ = now;
    if (dt <= 0.0) {
      dt = 0.05;
    }

    if (!have_pose_) {
      publishVelocity(Eigen::Vector3d::Zero());
      return;
    }

    if (!taking_off_ &&
        reachedCurrentWaypoint() &&
        current_wp_index_ < waypoints_.size() - 1)
    {
      current_wp_index_++;
      RCLCPP_INFO(this->get_logger(), "Advancing to waypoint %zu",
                  current_wp_index_);
    }

    Eigen::Vector3d target = computeTarget();
    Eigen::Vector3d cmd_vel;

    if (taking_off_) {
      // Simple proportional climb, same as before -- MPPI/scheduler
      // are reserved for waypoint tracking, not the vertical ramp.
      cmd_vel = Eigen::Vector3d(0.0, 0.0,
        mppi_controller::math_utils::clamp(
          (target.z() - drone_state_.position.z()) * 1.0, -1.0, 1.0));
    } else {
      // ---- Compute-aware MPPI call ----
      int used_n = next_n_;
      auto t0 = std::chrono::steady_clock::now();
      cmd_vel = mppi_.computeVelocityCommandWithN(drone_state_, target, used_n);
      auto t1 = std::chrono::steady_clock::now();
      double call_duration_sec = std::chrono::duration<double>(t1 - t0).count();

      next_n_ = scheduler_.recommendNextSampleCount(call_duration_sec, used_n);

      if (scheduler_.inSafetyFallback()) {
        RCLCPP_WARN(this->get_logger(),
          "AnytimeScheduler: two consecutive deadline misses -- consider fallback controller");
      }

      if (call_count_ % 20 == 0) {
        RCLCPP_INFO(this->get_logger(),
          "MPPI N=%d call_time=%.2fms next_N=%d",
          used_n, call_duration_sec * 1000.0, next_n_);
      }
      call_count_++;
      // ----------------------------------
    }

    publishVelocity(cmd_vel);

    if ((now - last_request_time_).seconds() > 5.0) {
      last_request_time_ = now;
      if (current_state_.mode != "OFFBOARD" && setpoints_sent_ > 20) {
        requestOffboard();
      } else if (!current_state_.armed && setpoints_sent_ > 20) {
        requestArm();
      }
    }
    setpoints_sent_++;
  }

  void requestOffboard()
  {
    if (!set_mode_client_->service_is_ready()) {return;}
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "OFFBOARD";
    set_mode_client_->async_send_request(req,
      [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        if (future.get()->mode_sent) {
          RCLCPP_INFO(this->get_logger(), "OFFBOARD mode requested");
        }
      });
  }

  void requestArm()
  {
    if (!arming_client_->service_is_ready()) {return;}
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    arming_client_->async_send_request(req,
      [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        if (future.get()->success) {
          RCLCPP_INFO(this->get_logger(), "Vehicle armed");
        }
      });
  }

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State current_state_;
  DroneState drone_state_;
  bool have_pose_ = false;

  std::vector<std::array<double, 3>> waypoints_;
  size_t current_wp_index_;
  double position_tolerance_ = 0.3;

  MppiController mppi_;
  AnytimeScheduler scheduler_;
  int next_n_;

  bool taking_off_ = true;
  Eigen::Vector3d ground_position_{0.0, 0.0, 0.0};
  rclcpp::Time takeoff_start_time_;
  double takeoff_duration_ = 3.0;
  double hover_altitude_ = 5.0;

  rclcpp::Time last_loop_time_;
  rclcpp::Time last_request_time_;
  int setpoints_sent_ = 0;
  int call_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardNode>());
  rclcpp::shutdown();
  return 0;
}
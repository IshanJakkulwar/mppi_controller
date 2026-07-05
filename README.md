# mppi_controller

Compute-aware (Anytime) Model Predictive Path Integral control for autonomous UAVs, built on ROS2 Humble + PX4 SITL + Gazebo Harmonic.

## Status

Actively in development. Current stage: baseline MPPI flying in Gazebo (fixed sample count), with a standalone compute-aware scheduler validated offline. Not yet wired into a live flight.

## Architecture

Gazebo -> PX4 -> MAVROS -> offboard_node -> Controller -> MAVROS setpoint

The ROS2 node never performs control math; it only subscribes, publishes, and calls controller classes. All optimization lives in `include/mppi_controller/controllers/`.

## Build

    cd ~/your_ws
    colcon build --packages-select mppi_controller
    source install/setup.bash

## Run

    ros2 run mppi_controller offboard_node

Requires PX4 SITL + Gazebo Harmonic + MAVROS already running.

## Package layout

    include/mppi_controller/
      controllers/   -- PID, MPC, MPPI, anytime scheduler
      models/        -- drone state representation, kinematic prediction model
      utils/         -- math helpers, sampling
    src/             -- implementations + standalone test executables

## Roadmap

- [x] Waypoint publisher
- [x] PID controller
- [x] Kinematic drone model
- [x] Linear MPC (gradient-descent based; QP solver upgrade pending if used in final results)
- [x] Baseline MPPI (standalone validated + flown)
- [x] Compute-aware sample scheduling (standalone validated)
- [ ] Compute-aware MPPI flown in Gazebo
- [ ] CPU-load stress testing
- [ ] Hardware testing (F450 / X500)

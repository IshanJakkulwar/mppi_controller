# mppi_controller

Compute-aware (Anytime) Model Predictive Path Integral control for autonomous UAVs, built on ROS2 Humble + PX4 SITL + Gazebo Harmonic.

## Status

Actively in development. Current stage: Beta Compute Aware MPPI flying in Gazebo (fixed vs adaptive sample count), with a compute-aware scheduler partially validated in SITL. Timestamped comparison for both in progress.

## Architecture

Gazebo -> PX4 -> MAVROS -> offboard_node -> Controller -> MAVROS setpoint

The ROS2 node never performs control math; it only subscribes, publishes, and calls controller classes. All optimization lives in `include/mppi_controller/controllers/`.

## Build

    cd ~/your_ws
    colcon build --packages-select mppi_controller
    source install/setup.bash

## Run

    ros2 run mppi_controller offboard_node                                             # adaptive
    ros2 run mppi_controller offboard_node --ros-args -p use_scheduler:=false          # fixed N=400
    ros2 run mppi_controller offboard_node --ros-args -p use_scheduler:=false -p fixed_n:=330  # constant-N control

Requires PX4 SITL + Gazebo Harmonic + MAVROS already running.

Automated Phase 2 trials + analysis (see PROJECT_PLAN.md). Run from this package's root
(`~/mppi_research_ws/src/mppi_controller`) so `results/` lands inside the repo:

    python3 scripts/run_trial.py --condition adaptive --trials 3
    python3 scripts/analyze_trials.py results/

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
- [X] Compute-aware MPPI flown in Gazebo
- [X] CPU-load stress testing
- [ ] Hardware testing (F450 / X500)

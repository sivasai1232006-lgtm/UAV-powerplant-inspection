# Simulation and UAV speed settings

Gazebo's world physics now requests `real_time_factor=2.0`. The original
`max_step_size=0.004` seconds remains unchanged. Since the existing update rate
of 250 Hz capped the product at 1.0, the world update rate is 500 Hz, which
permits a 2.0 upper bound. Gazebo still determines the achieved factor from
available CPU/rendering capacity.

The existing `/clock` Gazebo-to-ROS bridge is unchanged, and mission, planner,
controller, and allocator nodes use simulation time. PX4 SITL continues to use
the Gazebo simulator clock through its Gazebo integration and Micro XRCE-DDS
transport. Sensor rates and PX4 source were not changed.

The controller publishes PX4 `TrajectorySetpoint` velocity fields while a
waypoint is active. The configurable `horizontal_velocity_multiplier` is
applied once to the XY command limit in `uav_control/config/uav_param.yaml`.
The command direction is preserved; vertical setpoints retain the original
per-goal cap. The controller caps horizontal output at `max_velocity_limit`
(5 m/s). The mission's 2.0 m/s goal therefore produces at most 2.4 m/s. The
allocator's travel-time estimate was updated to 2.4 m/s. Battery estimation is
distance-based, so the multiplier changes estimated flight time but not energy
per meter.

The checked PX4 source (`v1.18.0-beta1-834-gb3e343c153`) defaults
`MPC_XY_VEL_MAX` to 12 m/s, `MPC_ACC_HOR` to 3 m/s^2, and `MPC_ACC_HOR_MAX` to
5 m/s^2. The intended 2.4 m/s is below those limits; no PX4 parameters or source
were changed. The PX4 runtime can have persisted parameter overrides.

During the local two-UAV Gazebo/PX4 SITL smoke run, `/world/powerplant/stats`
reported an RTF close to 2.0 (samples were approximately 1.99–2.00), with the
4 ms physics step retained. Both controllers armed and PX4 reported takeoff.
The inspection task path did not complete because Nav2 rejected the start pose
as being in lethal costmap space. That existing navigation problem prevents a
full mission, landing, and in-flight horizontal speed validation in this run.

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
waypoint is active. Horizontal and vertical commands use the configured
`velocity_gain` and are capped by the active goal's velocity limit; horizontal
commands are also capped by `max_velocity_limit` (5 m/s). The allocator uses
its configured `nominal_speed_mps` directly for travel-time estimates.
Battery estimation is distance-based.

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

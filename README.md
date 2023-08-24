# bicycle_ros_packages
all important ros packages for my research
1. **MPC_ros_node**: Node for the parallel MPC algorithm, consists of a cuda file and another main.c
2. **balance_command_node**: Node used for testing the performance of different balance controllers in Gazebo, where it publishes a command mentioned in the thesis.
3. **bicycle_lmi_controller**: Low-level balancing controller for the physical model, using LPV control and solving K gain with LMI mothods.
4. **bicycle_pid_controller**: Low-level balancing controller for the physical model, using PD control for balance.
5. **bicycle_ros**: Node for generating the simulation environment (launches Rviz/Gazebo as well as all necessary models).
6. **bike_experiment_viz**: Node for outdoor experiment, launches EKF, MPC, object detection.... Also contains launch file for rosbag visualization.
7. **multiple-object-tracking-lidar**: Modified package for point-cloud clustering and object tracking, fixed object display issues and introduced object growth/decay.
8. **odometry_tf**: TF transform broadcaster for odometry (for proper display in Rviz).
9. **px4_ekf**: EKF algorithm for bicycle, using sensor measurements of 2 GPS's, IMU and encoder.
10. **ti_mmwave_rospkg**: Package from ti's website, used to launch the mmwave radar. Config files are added so that the radar can operate according to current needs.

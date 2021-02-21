# create_control_ros
A ROS package to control iRobot create simulated in Gazebo.
ROS package prepared to insert your algorithm for controling simulated iRobot create.

You can use a C++ "SDK" or Python "SDK"

To start just pull the package and dependency packages as stated below.
Use "SDK" of your choice by editing create_control.cpp or create_control_py.py

Robot is controled with topic cmd_vel with member parameters:
geometry_msgs::Twist cmd_vel
cmd_vel.angular.x = 0
cmd_vel.angular.y = 0
cmd_vel.angular.z = 0
cmd_vel.linear.x = 0
cmd_vel.linear.y = 0
cmd_vel.linear.z = 0

Robot publishes values from sensors on topic sensors with defined message and member parameters:
create_control::create_sensors sensors
sensors.dist1 = 0 - (meters) average distance of obstacle from left front left laser scanner (simulated ultrasound sensor)
sensors.dist2 = 0 - all consecutive sensor in clockwise direction
sensors.dist3 = 0
sensors.dist4 = 0
sensors.dist5 = 0
sensors.dist6= 0
sensors.dist7 = 0
sensors.dist8 = 0
sensors.imu_p = 0 - angle around x axis (radian)
sensors.imu_r = 0 - angle around y axis
sensors.imu_y = 0 - angle around z axis
sensors.left_wheel = 0 - angle of left wheel (radian)
sensors.right_wheel = 0 - angle of right wheel (radian)

To run the simulation you can choose from a variety of launch files running c++ or python control, only simulation, only control:
control.launch
sim_control.launch
sim.launch
control.launch
sim_control.launch
sim.launch

control.launch also runs Rviz to visualize the robot and rqt displaying pictures representing time since the robot left starting tile which stops counting when the robot reaches goal tiles. This time is also saved in a results file in results directory.

Further you are required to pull related packages:
Smadas/irobot_create_ros - urdf, sdf robot description
Smadas/maze_generator_ros - maze generator generating urdf and sdf maze model and lauchfile spawning the robot in starting position

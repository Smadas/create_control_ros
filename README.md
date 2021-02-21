# create_control_ros
A ROS package to control iRobot create simulated in Gazebo.
ROS package prepared to insert your algorithm for controling simulated iRobot create.

You can use a C++ "SDK" or Python "SDK"

To start just pull the package and dependency packages as stated below.
Use "SDK" of your choice by editing create_control.cpp or create_control_py.py

Robot is controled with topic cmd_vel with member parameters:
<br>geometry_msgs::Twist cmd_vel
<br>cmd_vel.angular.x = 0
<br>cmd_vel.angular.y = 0
<br>cmd_vel.angular.z = 0
<br>cmd_vel.linear.x = 0
<br>cmd_vel.linear.y = 0
<br>cmd_vel.linear.z = 0

Robot publishes values from sensors on topic sensors with defined message and member parameters:
<br>create_control::create_sensors sensors
<br>sensors.dist1 = 0 - (meters) average distance of obstacle from left front left laser scanner (simulated ultrasound sensor)
<br>sensors.dist2 = 0 - all consecutive sensor in clockwise direction
<br>sensors.dist3 = 0
<br>sensors.dist4 = 0
<br>sensors.dist5 = 0
<br>sensors.dist6= 0
<br>sensors.dist7 = 0
<br>sensors.dist8 = 0
<br>sensors.imu_p = 0 - angle around x axis (radian)
<br>sensors.imu_r = 0 - angle around y axis
<br>sensors.imu_y = 0 - angle around z axis
<br>sensors.left_wheel = 0 - angle of left wheel (radian)
<br>sensors.right_wheel = 0 - angle of right wheel (radian)

To run the simulation you can choose from a variety of launch files running c++ or python control, only simulation, only control:
<br>control.launch
<br>sim_control.launch
<br>sim.launch
<br>control.launch
<br>sim_control.launch
<br>sim.launch

control.launch also runs Rviz to visualize the robot and rqt displaying pictures representing time since the robot left starting tile which stops counting when the robot reaches goal tiles. This time is also saved in a results file in results directory.

Further you are required to pull related packages:
<br>Smadas/irobot_create_ros - urdf, sdf robot description
<br>Smadas/maze_generator_ros - maze generator generating urdf and sdf maze model and lauchfile spawning the robot in starting position

Just make sure that your workspace is called catknin_ws otherwise change it in the code.

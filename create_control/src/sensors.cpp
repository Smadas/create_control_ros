//
// Created by adam on 14. 2. 2021.
//
#include "ros/ros.h" // hlavičkový súbor ROS funkcionalita
#include "sensor_msgs/LaserScan.h" // hlavičkový súbor pre správu LaserScan
#include "create_control/create_sensors.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"
#include "tf/tf.h"
#include "std_msgs/Time.h"
#include "ros/time.h"

//looking for home directory
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>

//random number
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

//files libraries
#include <iostream>
#include <fstream>

#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>

create_control::create_sensors sensors;
gazebo_msgs::ModelStates create_states;
sensor_msgs::JointState jointState;
bool create_states_initialized = false;
int create_model_index = 0;

std::string get_directory(std::string file_name, std::string file_num, std::string file_type)
{
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string file_directory;
    file_directory = homedir;
    file_directory = file_directory + "/catkin_ws/src/create_control_ros/" + file_name + file_num + "." + file_type;
    return file_directory;
}

double getRange(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int counter = 0;
    double range = 0;
    for (double angle = msg->angle_min; angle <= msg->angle_max; angle = angle + msg->angle_increment)
    {
        if (!isinf(msg->ranges.at(counter)))
        {
            range = range + msg->ranges.at(counter);
        }
        counter++;
    }
    if (range == 0)
    {
        range = 1000;
    }
    return range/(double)counter;
}

// callback funkcia, ktorá sa vyvolá vždy pri príchode novej správy na topic
// ako argument je smerník na obsah správy
void ultrasonicCallback1(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensors.dist1 = getRange(msg);
}
void ultrasonicCallback2(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensors.dist2 = getRange(msg);
}
void ultrasonicCallback3(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensors.dist3 = getRange(msg);
}
void ultrasonicCallback4(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensors.dist4 = getRange(msg);
}
void ultrasonicCallback5(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensors.dist5 = getRange(msg);
}
void ultrasonicCallback6(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensors.dist6 = getRange(msg);
}
void ultrasonicCallback7(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensors.dist7 = getRange(msg);
}
void ultrasonicCallback8(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensors.dist8 = getRange(msg);
}
void createStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    create_states = *msg;
    for (int i = 0; i < create_states.name.size(); i++)
    {
        if (create_states.name.at(i) == "create")
        {
            create_model_index = i;
        }
    }

    tf::Quaternion q(
            create_states.pose.at(create_model_index).orientation.x,
            create_states.pose.at(create_model_index).orientation.y,
            create_states.pose.at(create_model_index).orientation.z,
            create_states.pose.at(create_model_index).orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(sensors.imu_r, sensors.imu_p, sensors.imu_y);
    create_states_initialized = true;
}
void createJointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    jointState = *msg;
    sensors.right_wheel = jointState.position.at(1);
    sensors.left_wheel = jointState.position.at(0);
}
int main(int argc, char **argv) // main funkcia so vstupnými argumentami
{
    // registrácia novej node v roscore
    ros::init(argc, argv, "sensors");
    // objekt ktorým sa dá pristupovať k node API
    ros::NodeHandle n;

    // get drive time parameters
    int timer_state = 0;
    bool inside_start = false;
    bool inside_goal = false;
    double start_area_x1 = 0;
    double start_area_y1 = 0;
    double start_area_x2 = 0;
    double start_area_y2 = 0;
    double goal_area_x1 = 0;
    double goal_area_y1 = 0;
    double goal_area_x2 = 0;
    double goal_area_y2 = 0;
    n.getParam("start_area/x1",start_area_x1);
    n.getParam("start_area/y1",start_area_y1);
    n.getParam("start_area/x2",start_area_x2);
    n.getParam("start_area/y2",start_area_y2);
    n.getParam("goal_area/x1",goal_area_x1);
    n.getParam("goal_area/y1",goal_area_y1);
    n.getParam("goal_area/x2",goal_area_x2);
    n.getParam("goal_area/y2",goal_area_y2);

    // initialization drive timer
    std_msgs::Time drive_time_msg;
    ros::Time init_time = ros::Time::now();
    ros::Time now_time = ros::Time::now();
    cv::Mat time_img(cv::Size(150, 50), CV_8SC3, cv::Scalar(0));
    sensor_msgs::Image drive_time_img;
    cv_bridge::CvImage img_bridge;
    int time_counter = 0;
    namedWindow( "Display drive time", cv::WINDOW_AUTOSIZE );
    std::string drive_time_text;
    drive_time_text = "autobus";

    // open results sheet - drive time
    std::ofstream results_sheet_file;
    std::string results_sheet_path;
    results_sheet_path = get_directory("create_control/results/results_sheet", "", "txt");
    std::cout << results_sheet_path << std::endl;
    remove(results_sheet_path.c_str()); //removing the old file
    results_sheet_file.open (results_sheet_path.c_str());

    sensors.dist1 = 0;
    sensors.dist2 = 0;
    sensors.dist3 = 0;
    sensors.dist4 = 0;
    sensors.dist5 = 0;
    sensors.dist6= 0;
    sensors.dist7 = 0;
    sensors.dist8 = 0;
    sensors.imu_p = 0;
    sensors.imu_r = 0;
    sensors.imu_y = 0;
    sensors.left_wheel = 0;
    sensors.right_wheel = 0;

    // ohlásenie subscribera na topic "chatter" v roscore
    // buffer o veľkosti 1 správ, názov callback funkcie
    ros::Subscriber sub1 = n.subscribe("ultrasonic1", 1, ultrasonicCallback1);
    ros::Subscriber sub2 = n.subscribe("ultrasonic2", 1, ultrasonicCallback2);
    ros::Subscriber sub3 = n.subscribe("ultrasonic3", 1, ultrasonicCallback3);
    ros::Subscriber sub4 = n.subscribe("ultrasonic4", 1, ultrasonicCallback4);
    ros::Subscriber sub5 = n.subscribe("ultrasonic5", 1, ultrasonicCallback5);
    ros::Subscriber sub6 = n.subscribe("ultrasonic6", 1, ultrasonicCallback6);
    ros::Subscriber sub7 = n.subscribe("ultrasonic7", 1, ultrasonicCallback7);
    ros::Subscriber sub8 = n.subscribe("ultrasonic8", 1, ultrasonicCallback8);
    ros::Subscriber sub_states = n.subscribe("/gazebo/model_states", 1, createStatesCallback);
    ros::Subscriber sub_joints = n.subscribe("/joint_states", 1, createJointCallback);

    ros::Publisher pub = n.advertise<create_control::create_sensors>("sensors", 1);
    ros::Publisher pub_time = n.advertise<std_msgs::Time>("drive_timer", 1);
    ros::Publisher pub_time_img = n.advertise<sensor_msgs::Image>("drive_time_img", 1);

    // inicializácia sleep funkcie pre špecifickú frekvenciu vykonávania
    ros::Rate loop_rate(30); // fekvencia 10Hz
    // nekonečná slučka kým je spustený roscore
    while (ros::ok())
    {
        // kontrola všetkých callback funkcií a service-ov
        ros::spinOnce();

        pub.publish(sensors);
        // drive timer
        if (create_states_initialized) {
            // is inside start
            if ((create_states.pose.at(create_model_index).position.x > start_area_x1) &&
                (create_states.pose.at(create_model_index).position.x < start_area_x2)) {
                if ((create_states.pose.at(create_model_index).position.y > start_area_y1) &&
                    (create_states.pose.at(create_model_index).position.y < start_area_y2)) {
                    inside_start = true;
                    timer_state = 0;
                } else {
                    inside_start = false;
                    if (timer_state == 0)
                    {
                        timer_state = 1;
                    }
                }
            } else {
                inside_start = false;
                if (timer_state == 0)
                {
                    timer_state = 1;
                }
            }
            // is inside goal
            if ((create_states.pose.at(create_model_index).position.x > goal_area_x1) &&
                (create_states.pose.at(create_model_index).position.x < goal_area_x2)) {
                if ((create_states.pose.at(create_model_index).position.y > goal_area_y1) &&
                    (create_states.pose.at(create_model_index).position.y < goal_area_y2)) {
                    inside_goal = true;
                    if (timer_state == 1)
                    {
                        timer_state = 2;
                        // put drive time to file
                        int minutes = drive_time_msg.data.sec/60;
                        int seconds = drive_time_msg.data.sec - minutes*60;
                        results_sheet_file << ros::Time::now().sec << ":" << ros::Time::now().nsec << "::" << minutes << ":" << seconds << ":" << drive_time_msg.data.nsec << std::endl;
                    }
                } else {
                    inside_goal = false;
                }
            } else {
                inside_goal = false;
            }
            if (inside_start) {
                init_time = ros::Time::now();
            }
            // robot driving update timer
            if (!inside_goal && !inside_start) {
                now_time = ros::Time::now();
                if (init_time.nsec > now_time.nsec) {
                    drive_time_msg.data.nsec = init_time.nsec - now_time.nsec;
                    drive_time_msg.data.sec = now_time.sec - init_time.sec - 1;
                } else {
                    drive_time_msg.data.nsec = now_time.nsec - init_time.nsec;
                    drive_time_msg.data.sec = now_time.sec - init_time.sec;
                }

                pub_time.publish(drive_time_msg);

                // get minutes
                int minutes = drive_time_msg.data.sec/60;
                int seconds = drive_time_msg.data.sec - minutes*60;

                // draw timer
                //cv::putText(, vectorStr, textPoint, cv::FONT_HERSHEY_SIMPLEX, FONT_SCALE, VECTOR_COLOR, FONT_THICKNESS, 8, false);
                drive_time_text = std::to_string(minutes) + ":" + std::to_string(seconds) + ":" + std::to_string(drive_time_msg.data.nsec/1000000);
                time_img = cv::Mat::zeros(cv::Size(150, 50), CV_8SC3);
                cv::putText(time_img, //target image
                            drive_time_text, //text
                            cv::Point(5, time_img.rows / 1.7), //top-left position
                            cv::FONT_HERSHEY_DUPLEX,
                            1.0,
                            CV_RGB(0, 0, 255), //font color
                            2);

                std_msgs::Header header; // empty header
                if (time_counter >= (INT_MAX - 1))
                {
                    time_counter = 0;
                }
                time_counter = time_counter + 1;
                header.seq = time_counter; // user defined counter
                header.stamp = ros::Time::now(); // time
                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, time_img);
                img_bridge.toImageMsg(drive_time_img); // from cv_bridge to sensor_msgs::Image
                pub_time_img.publish(drive_time_img);
//                imshow( "Display drive time", time_img );
//                cv::waitKey(0.01);
            }
        }
        // sleep s trvaním aby sa dosiahla frekvencia slučky 10 HZ
        loop_rate.sleep();
    }

    results_sheet_file.close();
    return 0;
}


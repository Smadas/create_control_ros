#include "ros/ros.h" // hlavičkový súbor ROS funkcionalita - header file for ROS functionality
#include "geometry_msgs/Twist.h" // hlavickovy subor prikaz pohybu - header file for movement command
#include "create_control/create_sensors.h" // hlavickovy subor pre senzory - header file for sensors
//* pridajte dalsie potrebne hlavickove subory *//
//* add other needed header files *//

create_control::create_sensors  sensors_read; // globalna premenna struktura senzory - global variable structure sensors
// dist1, dist2, dist3, dist4, dis5, dis6, dis7, dist8, left_wheel, right_wheel, imu_r, imu_p, imu_y

// konstanty pre riadiaci algoritmus - constants for the control algorithm
#define MAX_SPEED 0.1
#define HALF_SPEED 0.25
#define ANG_SPEED 0.3
#define MIN_FRONT_DIST 0.1
//* pridajte vlastne konstanty *//
//* add your own constants *//

// funkcia pravidelne citajuca hodnoty senzorov - function regularly reading values of sensors
void ultrasonicCallback(const create_control::create_sensors ::ConstPtr& msg)
{
    sensors_read = *msg;
}

int main(int argc, char **argv) // main funkcia so vstupnými argumentami - main function with input arguments used by ROS
{
    // registrácia novej node v roscore - registration of new node in roscore
    ros::init(argc, argv, "simple_control");
    // objekt ktorým sa dá pristupovať k node API - object to access node API
    ros::NodeHandle n;
    // inicializácia sleep funkcie pre špecifickú frekvenciu vykonávania - initialize loop rate to 10 hz
    ros::Rate loop_rate(10); // fekvencia 10Hz
    // inicializacia spravy na zmenu rychlosti robota - initialization of message for changing of robot velocity
    geometry_msgs::Twist velocities;
    velocities.angular.x = 0;
    velocities.angular.y = 0;
    velocities.angular.z = 0;
    velocities.linear.x = 0;
    velocities.linear.y = 0;
    velocities.linear.z = 0;

    //* pridajte vlastne premenne pre potrebne pre svoj algoritmus *//
    //* add you own variables required for your algorithm *//

    // inicializacia citania hodnot zo senzorov - initialization of reading values from sensors
    ros::Subscriber sub_sensors = n.subscribe("/sensors", 1, ultrasonicCallback);

    // inicializacia odosielania zmeny rychlosti do robota - initialization of sending robot speeds
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // nekonečná slučka beziaca kým je spustený roscore - infinite loop running while roscore is running
    while (ros::ok())
    {
        // kontrola všetkých callback funkcií a service-ov - check of all callback functions and services
        ros::spinOnce();

        // riadiaci algoritmus DEMO - vpred, ak je prekazka zatoc vpravo a pokracuj - control algorithm DEMO - drive straight if obstacle turn right and continue
        // START riadiaci algoritmus - control algorithm
        //* vymente algorimus za vas algorimus *//
        //* change algorithm to your own *//
        velocities.linear.x = MAX_SPEED;
        velocities.angular.z = 0;

        if (sensors_read.dist2 < MIN_FRONT_DIST)
        {
            velocities.linear.x = 0;
            velocities.angular.z = ANG_SPEED;
        }
        if (sensors_read.dist3 < MIN_FRONT_DIST)
        {
            velocities.linear.x = 0;
            velocities.angular.z = ANG_SPEED;
        }
        // END riadiaci algoritmus - control algorithm

        // publikovanie zmenenych rychlosti do robota - publishing of changed robot velocities
        pub_vel.publish(velocities);

        // sleep s trvaním aby sa dosiahla frekvencia slučky 10 HZ - sleep of length to get 10Hz
        loop_rate.sleep();
    }

    return 0;
}

#!/usr/bin/env python
# license BSD

# import potrebnych kniznic - import of required libraries
import rospy # API ros python
from geometry_msgs.msg import Twist # sprava pre rychlosti robota - message for robot velocities
from create_control.msg import create_sensors # sprava pre hodnoty senzorov - message fro sensors values
##* pridajte dalsie potrebne kniznice *##
##* add other required libraries *##

sensors_read = create_sensors() # deklaracia struktury hodnot zo senzorov - declaration of structure of values of sensors
#dist1, dist2, dist3, dist4, dis5, dis6, dis7, dist8, left_wheel, right_wheel, imu_r, imu_p, imu_y

# funkcia pravidelne citajuca hodnoty zo senzorov - function regularly reading values from sensors
def sensor_callback(data):
    sensors_read.dist1 = data.dist1
    sensors_read.dist2 = data.dist2
    sensors_read.dist3 = data.dist3
    sensors_read.dist4 = data.dist4
    sensors_read.dist5 = data.dist5
    sensors_read.dist6 = data.dist6
    sensors_read.dist7 = data.dist7
    sensors_read.dist8 = data.dist8
    sensors_read.left_wheel = data.left_wheel
    sensors_read.right_wheel = data.right_wheel
    sensors_read.imu_r = data.imu_r
    sensors_read.imu_p = data.imu_p
    sensors_read.imu_y = data.imu_y

# funkcia hlavnej slucky na inicializaciu programu a pravidelne vykonavanie riadiaceho algoritmu
# function of main loop for initialization of program and regular execution of the control algorithm
def main_loop():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # registracia pravidelneho odosielania rychlosti do robota - registration of regular sending of velocities to robot
    rospy.Subscriber("sensors", create_sensors, sensor_callback) # registracia pravidelneho citania hodnot senzorov - registration of regular reading of values from sensors
    rospy.init_node('simple_control_py', anonymous=True) # registraci programu v roscore - registration of program in roscore

    rate = rospy.Rate(10) # nastavenie hlavnej slucky na 10 Hz - set main loop to 10Hz

    # inicializacia konstant pre riadiaci algoritmus - initialization of constants for the control algorithm
    MAX_SPEED = 0.1
    HALF_SPEED = 0.25
    ANG_SPEED = 0.3
    MIN_FRONT_DIST = 0.1
    ##* pridajte vlastne konstanty pre algoritmus *##
    ##* add your own constants for your algorithm *##

    ##* pridajte vlastne premenne pre potrebne pre svoj algoritmus *##
    ##* add you own variables required for your algorithm *##

    # hlavna slucka riadiaceho algoritmu pokym je roscore zapnuty- main loop of the control algorithm while roscore is on
    while not rospy.is_shutdown():
        # inicializacia spravy prikazu rychlosti pre robot - init message velocities of robot command
        velocities = Twist()
        velocities.angular.x = 0
        velocities.angular.y = 0
        velocities.angular.z = 0
        velocities.linear.x = 0
        velocities.linear.y = 0
        velocities.linear.z = 0

        # riadiaci algoritmus DEMO - vpred, ak je prekazka zatoc vpravo a pokracuj - control algorithm DEMO - drive straight if obstacle turn right and continue
        # START riadiaci algoritmus - control algorithm
        ##* vymente algorimus za vas algorimus *##
        ##* change algorithm to your own *##
        velocities.linear.x = MAX_SPEED
        velocities.angular.z = 0

        if sensors_read.dist2 < MIN_FRONT_DIST:
            velocities.linear.x = 0
            velocities.angular.z = ANG_SPEED
        if sensors_read.dist3 < MIN_FRONT_DIST:
            velocities.linear.x = 0
            velocities.angular.z = ANG_SPEED
        # END riadiaci algoritmus - control algorithm

        # publikovanie zmenenych rychlosti do robota - publishing of changed robot velocities
        pub.publish(velocities)

        # sleep s trvaním aby sa dosiahla frekvencia slučky 10 HZ - sleep of length to get 10Hz
        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
/* 
 * Author: Joshua Petrin  <lol@vanderbilt.edu>      
 */
#include <string>

// #include <curses.h>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <swiftpro/SwiftproState.h>
#include <swiftpro/position.h>
#include <swiftpro/vector.h>
#include <swiftpro/angle4th.h>

#include <swiftpro/position_change.h>


serial::Serial _serial;  // serial object



int main(int argc, char** argv)
{   
    ros::init(argc, argv, "swiftpro_teleop_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<swiftpro::vector>("teleop_vector_topic", 1);

    while (ros::ok())
    {
        swiftpro::vector vec;

        vec.dx = 0;

        pub.publish(vec);
        ros::spinOnce();
    }
    
    return 0;
}



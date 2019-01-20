/** 
 * Author: Joshua Petrin  <lol@vanderbilt.edu>      
 */
#include <string>

// #include <curses.h>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <pnr_ros_base/SwiftproState.h>
#include <pnr_ros_base/position.h>
#include <pnr_ros_base/vector.h>
#include <pnr_ros_base/angle4th.h>

//#include <pnr_ros_base/position_change.h>


serial::Serial _serial;  // serial object



int main(int argc, char** argv)
{   
    ros::init(argc, argv, "swiftpro_teleop_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<pnr_ros_base::vector>("teleop_vector_topic", 1);

    while (ros::ok())
    {
        pnr_ros_base::vector vec;

        vec.dx = 0;

        pub.publish(vec);
        ros::spinOnce();
    }
    
    return 0;
}



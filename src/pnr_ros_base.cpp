/* 
 * Software License Agreement:
 * do whatever teh crap you want with this software
 * just mention my name if you use it, bitte
 *
 * This node serves only to forward the cmd_vel publisher to the
 * uswift_vector_write topic. It does so by simply setting the Vector3
 * to the Vector3 Twist.linear.
 *
 * In the future, it will serve as the resource and message manager for
 * the project.
 * 
 * Author: Joshua Petrin <joshua.m.petrin@vanderbilt.edu>     
 */

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>

// #include <std_msgs/Bool.h> // ?
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <pnr_ros_base/uSwiftState.h>

// marker to update the vector command
bool update_cmd_vel = false;
geometry_msgs::Vector3 vector_out;

// ******
double scale = 5.0; // test to see what is best
// ******

void cmd_vel_callback(const geometry_msgs::Twist& msg_in)
{
  vector_out.x = scale * msg_in.linear.x;
  vector_out.y = scale * msg_in.linear.y;
  vector_out.z = scale * msg_in.linear.z;

  update_cmd_vel = true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pnr_ros_base");
  ros::NodeHandle nh;

  // write to pnr/uswift_vector_write
  ros::Publisher vector_writer
        = nh.advertise<geometry_msgs::Vector3>("pnr/uswift_vector_write", 1);

  ros::Subscriber cmd_vel = nh.subscribe("cmd_vel", 1, cmd_vel_callback);


  while (ros::ok())
  {
    if (update_cmd_vel) {
      vector_writer.publish(vector_out);
      update_cmd_vel = false;
    }

    ros::spinOnce();
  }

  return 0;
}

/* 
 * Software License Agreement:
 * do whatever teh crap you want with this software
 * just mention my name if you use it, bitte
 *
 * This node serves only to deconstruct and forward messages to their
 * necessary locations, such as the cmd_vel publisher to the
 * uswift_vector_write topic. Therefore, this node runs on the 
 * Raspberry Pi.
 *
 * In the future, it will serve as the resource and message manager for
 * the project.
 * 
 * Author: Joshua Petrin <joshua.m.petrin@vanderbilt.edu>     
 */

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
// #include <serial/serial.h>

// #include <std_msgs/Bool.h> // ?
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <pnr_ros_base/uSwiftState.h>

// marker to update the vector command
bool update_uswift_vector = false;
geometry_msgs::Vector3 uswift_vector_out;

// ******
// TODO: ROS parameterize
double scale = 5.0; // test to see what is best
// ******

void keyboard_teleop_callback(const geometry_msgs::Twist& msg_in)
{
  vector_out.x = scale * msg_in.linear.x;
  vector_out.y = scale * msg_in.linear.y;
  vector_out.z = scale * msg_in.linear.z;

  update_uswift_vector = true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pnr_ros_base");
  ros::NodeHandle nh;

  // write to pnr/uswift_vector_write
  ros::Publisher uswift_vector_writer
      = nh.advertise<geometry_msgs::Vector3>("pnr/uswift_vector_write", 1);

  ros::Subscriber cmd_vel
      = nh.subscribe("teleop_keyboard/twist", 1, keyboard_teleop_callback);


  while (ros::ok())
  {
    if (update_uswift_vector) {
      uswift_vector_writer.publish(uswift_vector_out);
      update_uswift_vector = false;
    }

    ros::spinOnce();
  }

  return 0;
}

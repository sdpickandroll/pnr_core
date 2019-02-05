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

#include <std_msgs/Bool.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

// #include <pnr_ros_base/uSwiftState.h>


// *********************
// Parameters
// *********************

// double on "/pnr_core/uswift_vector_scale"
double uswift_vector_scale;




// *********************
// <end Parameters>
// *********************



// marker to update the vector command
bool update_uswift_vector = false;
geometry_msgs::Vector3 uswift_vector_out;



void keyboard_teleop_callback(const geometry_msgs::Twist& msg_in)
{
  uswift_vector_out.x = uswift_vector_scale * msg_in.linear.x;
  uswift_vector_out.y = uswift_vector_scale * msg_in.linear.y;
  uswift_vector_out.z = uswift_vector_scale * msg_in.linear.z;

  update_uswift_vector = true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pnr_core");
  ros::NodeHandle nh("pnr_core");

  // write to /pnr_swiftpro/...
  ros::Publisher uswift_vector_writer
      = nh.advertise<geometry_msgs::Vector3>("/pnr_swiftpro/vector_write", 1);
  ros::Publisher uswift_actuator_writer
      = nh.advertise<std_msgs::Bool>("/pnr_swiftpro/actuator_write", 1);

  ros::Subscriber cmd_vel
      = nh.subscribe("/teleop_keyboard/twist", 1, keyboard_teleop_callback);

  // scale of the uswift_vector_scale (default = 1.0)
  if (!nh.hasParam("uswift_vector_scale"))
  {
    ROS_WARN("rosparam '/pnr_core/uswift_vector_scale' not found.");
    ROS_WARN("Defaulting to /pnr_core/uswift_vector_scale = 1.0.");
  }


  ros::spinOnce();

  while (ros::ok())
  {
    // update parameters
    nh.param<double>("/pnr_core/uswift_vector_scale", uswift_vector_scale, 1.0);
    
    if (update_uswift_vector)
    {
      uswift_vector_writer.publish(uswift_vector_out);
      update_uswift_vector = false;
    }

    ros::spinOnce();
  }

  return 0;
}

/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>      
 */
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <pnr_ros_base/SwiftproState.h>
#include <pnr_ros_base/status.h>
#include <pnr_ros_base/position.h>
#include <pnr_ros_base/angle4th.h>

//#include <pnr_ros_base/position_change.h>

serial::Serial _serial;             // serial object
pnr_ros_base::SwiftproState pos;
pnr_ros_base::SwiftproState desired_pos;

/* 
 * Description: callback when receive data from position_write_topic
 * Inputs:      msg(float)          3 cartesian coordinates: x, y, z(mm)
 * Outputs:     Gcode               send gcode to control swift pro
 */
void position_write_callback(const pnr_ros_base::position& msg)
{
    std::string Gcode("");
    std::string result;
    char x[10];
    char y[10];
    char z[10];

    desired_pos.x = msg.x;
    desired_pos.y = msg.y;
    desired_pos.z = msg.z;
    sprintf(x, "%.2f", msg.x);
    sprintf(y, "%.2f", msg.y);
    sprintf(z, "%.2f", msg.z);
    Gcode = std::string("G0 X") + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
    ROS_INFO("Gcode: %s\n", Gcode.c_str());
    _serial.write(Gcode.c_str());
    result = _serial.read(_serial.available());
}


void position_read_callback(const pnr_ros_base::SwiftproState& msg)
{
    pos = pnr_ros_base::SwiftproState(msg);
}


/**
 * New method. 
 * 
 * Description: callback when a control input is received.
 * Inputs:      msg(float)          3 cartesian diffs: dx, dy, dz(mm)
 * Outputs:     Gcode
 */
void t_vector_callback(const geometry_msgs::Twist& msg)
{
    // TODO:
    // PROVISIONAL!!! This shouldn't really be here. 
    // This was only for the 3D mouse, which likes sending a loooot of messages. 
    // Take this out when you run this next!!!!!
    static int trigger = 1;
    if (trigger >= 5)
    {
        std::string Gcode("");
        std::string result;
        char x[10];
        char y[10];
        char z[10];

        // TODO: Make sure these don't go out of bounds.
        desired_pos.x += msg.linear.x;
        desired_pos.y += msg.linear.y;
        desired_pos.z += msg.linear.z;

        sprintf(x, "%.2f", msg.linear.x);
        sprintf(y, "%.2f", msg.linear.y);
        sprintf(z, "%.2f", msg.linear.z);

        Gcode = std::string("G2204 X") + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
        ROS_INFO("%s", Gcode.c_str());
        _serial.write(Gcode.c_str());
        result = _serial.read(_serial.available());

        trigger = 1;
    }
    ++trigger;
}


/* 
 * Description: callback when receive data from angle4th_topic
 * Inputs:      msg(float)          angle of 4th motor(degree)
 * Outputs:     Gcode               send gcode to control swift pro
 */
void angle4th_callback(const pnr_ros_base::angle4th& msg)
{
    std::string Gcode = "";
    std_msgs::String result;
    char m4[10];
    
    desired_pos.motor_angle4 = msg.angle4th;
    sprintf(m4, "%.2f", msg.angle4th);
    Gcode = (std::string)"G2202 N3 V" + m4 + "\r\n";
    ROS_INFO("%s", Gcode.c_str());
    _serial.write(Gcode.c_str());
    result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from pnr_ros_base_status_topic
 * Inputs:      msg(uint8)          status of gripper: attach if 1; detach if 0
 * Outputs:     Gcode               send gcode to control swift pro
 */
void swiftpro_status_callback(const pnr_ros_base::status& msg)
{
    std::string Gcode = "";
    std_msgs::String result;

    if (msg.status == 1)
        Gcode = (std::string)"M17\r\n";   // attach
    else if (msg.status == 0)
        Gcode = (std::string)"M2019\r\n";
    else
    {
        ROS_INFO("Error:Wrong swiftpro status input");
        return;
    }
    
    desired_pos.swiftpro_status = msg.status;
    ROS_INFO("%s", Gcode.c_str());
    _serial.write(Gcode.c_str());
    result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from gripper_topic
 * Inputs:      msg(uint8)          status of gripper: work if 1; otherwise 0
 * Outputs:     Gcode               send gcode to control swift pro
 */
void gripper_callback(const pnr_ros_base::status& msg)
{
    std::string Gcode = "";
    std_msgs::String result;

    if (msg.status == 1)
        Gcode = (std::string)"M2232 V1" + "\r\n";
    else if (msg.status == 0)
        Gcode = (std::string)"M2232 V0" + "\r\n";
    else
    {
        ROS_INFO("Error:Wrong gripper status input");
        return;
    }
    
    desired_pos.gripper = msg.status;
    ROS_INFO("%s", Gcode.c_str());
    _serial.write(Gcode.c_str());
    result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from pump_topic
 * Inputs:      msg(uint8)          status of pump: work if 1; otherwise 0
 * Outputs:     Gcode               send gcode to control swift pro
 */
void pump_callback(const pnr_ros_base::status& msg)
{
    std::string Gcode = "";
    std_msgs::String result;

    if (msg.status == 1)
        Gcode = (std::string)"M2231 V1" + "\r\n";
    else if (msg.status == 0)
        Gcode = (std::string)"M2231 V0" + "\r\n";
    else
    {
        ROS_INFO("Error:Wrong pump status input");
        return;
    }
    
    desired_pos.pump = msg.status;
    ROS_INFO("%s", Gcode.c_str());
    _serial.write(Gcode.c_str());
    result.data = _serial.read(_serial.available());
}


/* 
 * Node name:
 *   swiftpro_write_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   swiftpro_state_topic
 *
 * Topic subscribe: (queue size = 1)
 *   robot_vector
 *   position_write_topic
 *   swiftpro_status_topic
 *   angle4th_topic
 *   gripper_topic
 *   pump_topic
 */
int main(int argc, char** argv)
{   
    ros::init(argc, argv, "swiftpro_write_node");
    ros::NodeHandle nh;
    pnr_ros_base::SwiftproState swiftpro_state;

    ros::Subscriber tsub = nh.subscribe("cmd_vel", 1, t_vector_callback);
    // ros::Subscriber tsub = nh.subscribe("teleop_vector_topic", 1, t_vector_callback);
    ros::Subscriber sub1 = nh.subscribe("position_write_topic", 1, position_write_callback);
    ros::Subscriber sub2 = nh.subscribe("swiftpro_status_topic", 1, swiftpro_status_callback);
    ros::Subscriber sub3 = nh.subscribe("angle4th_topic", 1, angle4th_callback);
    ros::Subscriber sub4 = nh.subscribe("gripper_topic", 1, gripper_callback);
    ros::Subscriber sub5 = nh.subscribe("pump_topic", 1, pump_callback);
    ros::Subscriber possub = nh.subscribe("SwiftproState_topic", 1, position_read_callback);
    // ros::Rate loop_rate(20);

    ros::Duration(3.5).sleep();  // wait 3.5s

    try
    {
        _serial.setPort("/dev/ttyACM0");
        _serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        _serial.setTimeout(to);
        _serial.open();
        ROS_INFO_STREAM("Port has been open successfully");
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    
    if (_serial.isOpen())
    {
        ros::Duration(3.5).sleep();             // wait 3.5s
        // _serial.write("M2120 V0\r\n");          // stop report position
        ros::Duration(0.1).sleep();             // wait 0.1s
        _serial.write("M17\r\n");               // attach
        ros::Duration(0.1).sleep();             // wait 0.1s
        ROS_INFO_STREAM("Attach and wait for commands");
    }

    while (ros::ok())
    {
        // pub.publish(desired_pos);
        ros::spin();
        // loop_rate.sleep();
    }
    
    return 0;
}



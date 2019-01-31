/* 
 * Software License Agreement
 * do whatever teh crap you want with this software
 * just mention my name if you use it, bitte
 * 
 * Author: Joshua Petrin <joshua.m.petrin@vanderbilt.edu>     
 */
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>

// #include <std_msgs/Bool.h> // ?
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <pnr_ros_base/uSwiftState.h>

/*
 * For reference...
 * The uSwift's error codes are the following:
 * 
 * E20 = Command does not exist
 * E21 = Parameter error
 * E22 = Address out of range
 * E23 = Command buffer full
 * E24 = Power unconnected
 * E25 = Operation failure
 */


// serial object, from the amazing ROS serial package
serial::Serial *usb;

// Too many vector commands will flood the uSwift.
// This variable is set to 'true' every 50 milliseconds.
bool accept_vector; 


void position_write_callback(const geometry_msgs::Point& msg_in)
{
    char x[8];
    char y[8];
    char z[8];
    sprintf(x, "%.3f", msg_in.x);
    sprintf(y, "%.3f", msg_in.y);
    sprintf(z, "%.3f", msg_in.z);

    std::string Gcode = std::string("G0 X") 
        + x + " Y" + y + " Z" + z + " F10000" + "\r\n";

    ROS_DEBUG("Sending a position command to the uSwift.\n"
        "Gcode: %s\n", Gcode.c_str());

    usb->write(Gcode.c_str());

    std::string result = usb->read(usb->available());
    if (result != "ok\n") // I think this is right
    {
        ROS_WARN("Received error from uSwift after the command %s:\n"
         "%s", Gcode.c_str(), result.c_str());
    }
}


void vector_write_callback(const geometry_msgs::Vector3& msg_in)
{
    if (accept_vector)
    {
        std::string Gcode;
        char x[8];
        char y[8];
        char z[8];

        // TODO: Make sure these don't go out of bounds.
        desired_pos.x += msg_in.linear.x;
        desired_pos.y += msg_in.linear.y;
        desired_pos.z += msg_in.linear.z;

        sprintf(x, "%.2f", msg_in.linear.x);
        sprintf(y, "%.2f", msg_in.linear.y);
        sprintf(z, "%.2f", msg_in.linear.z);

        Gcode = std::string("G2204 X")
            + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
        ROS_INFO("Sending vector command to the uSwift.\n"
            "Gcode: %s", Gcode.c_str());
        
        usb->write(Gcode.c_str());

        std::string result = usb->read(usb->available());
        if (result != "ok\n") // I think this is right
        {
            ROS_WARN("Received error from uSwift after the command %s:\n"
             "%s", Gcode.c_str(), result.c_str());
        }
    }
}


/**
 * Process a generic string that is coming from the uSwift
 * (in the form "ok \w# \w# \w#")
 * 
 * TODO: See how the data is formatted form the uSwift, and write
 * this function for to that format. 
 */
process_str()
{
    char str[2048];
    strcpy(str, data.c_str());
    char* pch = strtok(str, " ");
    float x, y, z, r;
    int vals = 0;


    while (vals < 4)
    {
        if (pch == NULL)
        {
            ROS_INFO_STREAM("Serial data misaligned. "
                "Defaulting to previous measurement.");
            return;
        }
        switch (pch[0])
        {
            case 'X':
                x = atof(pch+1);
                ++vals;
                break;
            case 'Y':
                y = atof(pch+1);
                ++vals;
                break;
            case 'Z':
                z = atof(pch+1);
                ++vals;
                break;
            case 'R':
                r = atof(pch+1);
                ++vals;
                break;
            case '@':
                break;
            default:
                ROS_INFO_STREAM("Serial data misaligned. "
                    "Defaulting to previous measurement.");
                return;
        }
        pch = strtok(NULL, " ");
    }

    state->motor_angle4 = r;
    state->x = x;
    state->y = y;
    state->z = z;
}



/**
 * The uswift_state_write sub can only receive either angle commands or
 * position commands, but not both.
 */
void state_write_callback(const pnr_ros_base::uSwiftState& msg_in)
{
    // Check to see if any angle changes are desired
    if (msg_in.angle_base_motor == 0 &&
        msg_in.angle_arm_motor == 0 &&
        msg_in.angle_hand_motor == 0 &&
        msg_in.angle_base_motor == 0)
    {
        geometry_msgs::Point& point;
        point.x = msg_in.x;
        point.y = msg_in.y;
        point.z = msg_in.z;

        position_write_callback(point);
    }
    else
    {
        std::string Gcode = std::string("G0 X") 
        + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
    }
    char x[8];
    char y[8];
    char z[8];
    sprintf(x, "%.3f", in_msg.x);
    sprintf(y, "%.3f", in_msg.y);
    sprintf(z, "%.3f", in_msg.z);

    std::string Gcode = std::string("G0 X") 
        + x + " Y" + y + " Z" + z + " F10000" + "\r\n";

    ROS_DEBUG("Sending position command to the uSwift.\n"
        "Gcode: %s\n", Gcode.c_str());

    usb->write(Gcode.c_str());
    std::string result = usb->read(usb->available());

    sprintf(m4, "%.2f", msg.angle4th);
    Gcode = (std::string)"G2202 N3 V" + m4 + "\r\n";
    ROS_INFO("%s", Gcode.c_str());
    _serial.write(Gcode.c_str());
    result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from swiftpro_status_topic
 * Inputs:      msg(uint8)          status of gripper: attach if 1; detach if 0
 * Outputs:     Gcode               send gcode to control swift pro
 */
void swiftpro_status_callback(const swiftpro::status& msg)
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
void gripper_callback(const swiftpro::status& msg)
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
void pump_callback(const swiftpro::status& msg)
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
    ros::init(argc, argv, "swiftpro_node");
    ros::NodeHandle nh;

    // complete state of the robot
    ros::Publisher us_state_pub
        = nh.advertise<pnr_ros_base::uswift_state>("uswift_state", 1);
    ros::Publisher us_pos_pub = nh.advertise<geometry_msgs::Point>("uswift_position", 1);
    ros::Publisher us_actuator_pub = nh.advertise<std_msgs::Bool>("uswift_actuator_on", 1);

    ros::Subscriber ste_sub = nh.subscribe("uswift_state_write", 1, state_write_callback);
    ros::Subscriber pos_sub = nh.subscribe("uswift_position_write", 1, position_write_callback);
    ros::Subscriber vec_sub = nh.subscribe("uswift_vector_write", 1, vector_write_callback);
    ros::Subscriber atr_sub = nh.subscribe("uswift_actuator_write", 1, state_actuator_callback);
    // ros::Rate loop_rate(20);

    ros::Duration(3.5).sleep();  // wait 3.5s (???)

    try
    {
        serial = new serial::Serial(std::string(name), 115200, serial::Timeout::simpleTimeout(1000));
        serial.setPort("/dev/ttyACM0");
        serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial.setTimeout(to);
        serial.open();
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



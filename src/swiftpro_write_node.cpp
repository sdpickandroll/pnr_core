/* 
 * Software License Agreement:
 * do whatever teh crap you want with this software
 * just mention my name if you use it, bitte
 * 
 * Author: Joshua Petrin <joshua.m.petrin@vanderbilt.edu>     
 */
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>

#include <std_msgs/Bool.h> // ?
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
// #include <geometry_msgs/Twist.h>

#include <pnr_ros_base/uSwiftState.h>

/*
 * For reference...
 * The uSwift's error codes are the following:
 * 
 * E20 = Command does not exist
 * E21 = Parameter error
 * E22 = Location out of range
 * E23 = Command buffer full
 * E24 = Power unconnected
 * E25 = Operation failure
 */


// Too many vector commands will flood the uSwift.
// This variable is set to 'true' every flood_secs seconds.
bool accept_vector = true;
double flood_secs = 0.05;

// how fast the move commands are executed on the uSwift
// (10000 is fastest)
int move_speed = 10000;

// time between how often this node publishes a new state (secs)
double pos_update_secs = 0.05;

// publishing clearence, true => ready to publish new state
bool publish_uswift_state = false;

// published values
pnr_ros_base::uSwiftState us_state;
geometry_msgs::Point us_pos;
std_msgs::Bool us_actuator_on;

// serial object, from the amazing ROS serial package
serial::Serial *usb;




void position_write_callback(const geometry_msgs::Point& msg_in)
{
    char x[8];
    char y[8];
    char z[8];
    char ms[8];
    sprintf(x, "%.2f", msg_in.x);
    sprintf(y, "%.2f", msg_in.y);
    sprintf(z, "%.2f", msg_in.z);
    sprintf(ms, "%i", move_speed);

    std::string Gcode = std::string("G0 X") 
        + x + " Y" + y + " Z" + z + " F" + ms + "\n";

    ROS_DEBUG("Sending a position command to the uSwift.\n"
        "Gcode: %s\n", Gcode.c_str());

    usb->write(Gcode.c_str());

    std::string result = usb->read(usb->available());
    if (result[0] == 'E')
    {
        ROS_WARN("Received error from uSwift after the command %s:\n"
         "%s", Gcode.c_str(), result.c_str());
    }
}


void cyl_position_write_callback(const geometry_msgs::Point& msg_in)
{
    // stretch, rotation, and height (aka: r, theta, z)
    char s[8];
    char r[8];
    char h[8];
    char ms[8];
    sprintf(s, "%.2f", msg_in.x);
    sprintf(r, "%.2f", msg_in.y);
    sprintf(h, "%.2f", msg_in.z);
    sprintf(ms, "%i", move_speed);


    std::string Gcode = std::string("G2201 S") 
        + s + " R" + r + " H" + h + " F" + ms + "\n";

    ROS_DEBUG("Sending a cylindrical pos command to the uSwift.\n"
        "Gcode: %s\n", Gcode.c_str());

    usb->write(Gcode.c_str());

    std::string result = usb->read(usb->available());
    if (result[0] == 'E')
    {
        ROS_WARN("Received error from uSwift after the command %s:\n"
         "%s", Gcode.c_str(), result.c_str());
    }
}


void vector_write_callback(const geometry_msgs::Vector3& msg_in)
{
    if (accept_vector)
    {
        char x[8];
        char y[8];
        char z[8];
        char ms[8];
        sprintf(x, "%.2f", msg_in.x);
        sprintf(y, "%.2f", msg_in.y);
        sprintf(z, "%.2f", msg_in.z);
        sprintf(ms, "%i", move_speed);


        std::string Gcode = std::string("G2204 X")
            + x + " Y" + y + " Z" + z + " F" + ms + "\n";
        ROS_INFO("Sending vector command to the uSwift.\n"
            "Gcode: %s", Gcode.c_str());
        
        usb->write(Gcode.c_str());

        std::string result = usb->read(usb->available());
        if (result[0] == 'E')
        {
            ROS_WARN("Received error from uSwift after the command %s:\n"
             "%s", Gcode.c_str(), result.c_str());
        }

        accept_vector = false;
    }
}

void cyl_vector_write_callback(const geometry_msgs::Vector3& msg_in)
{
    if (accept_vector)
    {
        char s[8];
        char r[8];
        char h[8];
        char ms[8];
        sprintf(s, "%.2f", msg_in.x);
        sprintf(r, "%.2f", msg_in.y);
        sprintf(h, "%.2f", msg_in.z);
        sprintf(ms, "%i", move_speed);

        std::string Gcode = std::string("G2205 S")
            + s + " R" + r + " H" + h + " F" + ms + "\n";
        ROS_INFO("Sending cyl.vector command to the uSwift.\n"
            "Gcode: %s", Gcode.c_str());
        
        usb->write(Gcode.c_str());

        std::string result = usb->read(usb->available());
        if (result[0] == 'E')
        {
            ROS_WARN("Received error from uSwift after the command %s:\n"
             "%s", Gcode.c_str(), result.c_str());
        }

        accept_vector = false;
    }
}


void actuator_write_callback(const std_msgs::Bool& msg_in)
{
    us_actuator_on.data = msg_in.data;

    // TODO:
    // Not sure if this should be M2231 or M2232...
    // Depends on whether we have the suction or the gripper installed
    std::string Gcode = "M2231 ";
    if (us_actuator_on.data)
        Gcode += "V1";
    else
        Gcode += "V0";
    Gcode += "\n"; // this is very important!!

    ROS_INFO("Sending actuator command to the uSwift.\n"
        "Gcode: %s", Gcode.c_str());
        
    usb->write(Gcode.c_str());

    std::string result = usb->read(usb->available());
    if (result[0] == 'E')
    {
        ROS_WARN("Received error from uSwift after the command %s:\n"
         "%s", Gcode.c_str(), result.c_str());
    }
}

/**
 * TODO: Document this confusing document (lol) 
 */
void state_read_callback(const ros::TimerEvent&)
{
    // I'm not sure if it's possible to get the state of 
    // all the joints from the uSwift, so for now, we'll just
    // do the actuator angle and the end effector position.

    std::string data = usb->read(usb->available());
    ROS_INFO_STREAM("Read from USB: " << data);
    // DEBUG
    // data = usb->read(usb->available());
    // ROS_INFO_STREAM("Read from USB: " << data);

    char str[200];
    strcpy(str, data.c_str());
    char* pch = strtok(str, " ");
    float x, y, z, r;

    int vals = 0;

    while (vals < 4)
    {
        if (pch == NULL)
        {
            ROS_INFO_STREAM("uSwift data misaligned.");
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
                ROS_INFO_STREAM("uSwift data misaligned.");
                return;
        }
        pch = strtok(NULL, " ");
    }

    // pnr_ros_base::uswift_state us_state
    us_state.x = x;
    us_state.y = y;
    us_state.z = z;
    us_state.angle_actuator = r;
    us_state.actuator = us_actuator_on.data;

    // geometry_msgs::Point us_pos
    us_pos.x = x;
    us_pos.y = y;
    us_pos.z = z;

    // std_msgs::Bool us_actuator_on
    // we assume that the state of the actuator is equal to our stored state

    // we can publish now
    publish_uswift_state = true;
}


void vector_accept_callback(const ros::TimerEvent&)
{
    accept_vector = true;
}


/* 
 * Node name:
 *   
 *
 * Topics published: (rate = 20Hz, queue size = 1)
 *   
 *
 * Topics subscribed: (queue size = 1)
 *   
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pnr_swiftpro_node");
    ros::NodeHandle nh;

    // set defaults for the messages
    us_actuator_on.data = false;

    // complete state of the robot
    ros::Publisher us_state_pub
        = nh.advertise<pnr_ros_base::uSwiftState>("uswift_state", 1);
    ros::Publisher us_pos_pub = 
        nh.advertise<geometry_msgs::Point>("pnr/uswift_position", 1);
    ros::Publisher us_actuator_on_pub = 
        nh.advertise<std_msgs::Bool>("uswift_actuator", 1);

    ros::Subscriber pos_sub = 
        nh.subscribe("uswift_position_write", 1, position_write_callback);
    ros::Subscriber pcy_sub = 
        nh.subscribe("uswift_cyl_position_write", 1, cyl_position_write_callback);
    ros::Subscriber vec_sub = 
        nh.subscribe("uswift_vector_write", 1, vector_write_callback);
    ros::Subscriber vcy_sub = 
        nh.subscribe("uswift_cyl_vector_write", 1, cyl_vector_write_callback);
    ros::Subscriber atr_sub = 
        nh.subscribe("uswift_actuator_write", 1, actuator_write_callback);
    // ros::Rate loop_rate(20);

    ros::Duration(3.5).sleep();  // wait 3.5s (???)

    try
    {
        usb = new serial::Serial(
            std::string("/dev/ttyACM0"), // TODO: Make a program that will test the ports for uSwift's
            115200, 
            serial::Timeout::simpleTimeout(1000));
        ROS_INFO_STREAM("uSwift's USB port has been opened successfully");
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open USB port for the uSwift!");
        ROS_ERROR_STREAM("Trace: " << e.what());
        return -1;
    }
    
    if (usb->isOpen())
    {
        ros::Duration(3.5).sleep();     // wait 3.5s
        // usb->write("M2120 V0\r\n");  // stop report position
        ros::Duration(0.1).sleep();     // wait 0.1s
        usb->write("M17\n");            // attach
        ROS_INFO_STREAM("uSwift attached and waiting for commands.");
        ros::Duration(1.0).sleep();     // wait 1s

        // Read from USB: @3 X86.8411 Y-0.1332 Z88.6395 R89.1000


        std::string result = usb->read(usb->available());
        ROS_INFO("uSwift startup message: \n%s", result.c_str());

        ros::Duration(0.5).sleep();       // wait 0.5s
        usb->write("G0 X86.8411 Y-0.1332 Z33.6395 F1000\n"); // move to the uncalibrated "home" position
                                          // TODO: Learn how to calibrate this mugger
                                          // Maybe have a command that sets home position??
        ros::Duration(0.01).sleep();      // wait 0.01s
        result = usb->read(usb->available());
        if (result[0] == 'E')
        {
            ROS_WARN("Received error from uSwift after the command %s:\n"
             "%s", "G0 X86.8411 Y-0.1332 Z33.6395 F1000\n", result.c_str());
        }

        ros::Duration(1.0).sleep();     // wait 1.0s

        // Ask for position updates every pos_update_secs seconds.
        char pus_msg[100];
        sprintf(pus_msg, "M2120 V%.3f\n", pos_update_secs);

        ROS_INFO("Sending feedback command to the uSwift.\n"
            "Gcode: %s", pus_msg);
        usb->write(pus_msg);
        ros::Duration(pos_update_secs).sleep(); // wait for an update?
        // ros::Duration(0.01).sleep();     // wait 0.01s
        result = usb->read(usb->available());
        if (result[0] == 'E')
        {
            ROS_WARN("Received error from uSwift after the command %s:\n"
             "%s", pus_msg, result.c_str());
        }


        // initialize and start timers

        // the uSwift's position update timer
        ros::Timer pos_update_timer 
            = nh.createTimer(ros::Duration(pos_update_secs), state_read_callback);

        // the vector timer update manager (ensure the vectors don't flood)
        ros::Timer vec_timer_manager
            = nh.createTimer(ros::Duration(flood_secs), vector_accept_callback);

        // program loop: wait for callbacks and publish when we are ready
        while (ros::ok())
        {
            if (publish_uswift_state)
            {
                us_state_pub.publish(us_state);
                us_pos_pub.publish(us_pos);
                us_actuator_on_pub.publish(us_actuator_on);
                publish_uswift_state = false;
            }
            ros::spinOnce();
        }
    }

    
    return 0;  // Трахни грудь моей женщины! 
}

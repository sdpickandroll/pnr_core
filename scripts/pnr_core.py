#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
# from std_msgs.msg import String
# from std_msgs.msg import Empty

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Joy



## Parameters
# /pnr_core/uswift_vector_scale
uswift_vector_scale = 25.0

# /pnr_core/roomba_vector_scale
roomba_vector_scale = 1.0

# /pnr_core/roomba_angular_scale
roomba_angular_scale = 1.0

# update period (seconds)
ROSPARAM_UPDATE_PERIOD = 0.5


## Publishers 
# (we put these declarations here to make them global)
uswift_vector_write = 0
uswift_actuator_write = 0
roomba_twist_write = 0


## Assorted global variables

# roomba flood protection
send_vector_roomba = True
flood_vector_roomba_thresh = 0.2

# is the user controlling the arm or the roomba?
control_roomba = True

# uswift vector
uswift_vector_out = Vector3()

# uswift actuator
uswift_actuator_out = Bool(False)

# roomba cmd_vel
cmd_vel = Twist()

# spacenav button globals
spacenav_b0_pressed = False
spacenav_b1_pressed = False

# xbox controller globals
xbox_a_pressed = False      # 0
xbox_b_pressed = False      # 1
xbox_x_pressed = False      # 2
xbox_y_pressed = False      # 3
xbox_lb_pressed = False     # 4
xbox_rb_pressed = False     # 5
xbox_back_pressed = False   # 6
xbox_start_pressed = False  # 7
xbox_power_pressed = False  # 8
xbox_lsb_pressed = False    # 9
xbox_rsb_pressed = False    # 10



## Callbacks

# teleop calllback
# does nothing fancy; just forwards the sparse messages to the uSwift
def keyboard_teleop_callback(twist):
    global uswift_vector_write
    global uswift_vector_out
    global uswift_vector_scale

    # rospy.loginfo('Publishing a uSwift vector')
    
    uswift_vector_out.x = uswift_vector_scale * twist.linear.x
    uswift_vector_out.y = uswift_vector_scale * twist.linear.y
    uswift_vector_out.z = uswift_vector_scale * twist.linear.z

    uswift_vector_write.publish(uswift_vector_out)



# actuator callback
# this is never actually called by a subscriber, but it is called by several 
# methods that are.
def actuator_write_callback(state):
    global uswift_actuator_write
    global uswift_actuator_out

    uswift_actuator_out = state
    uswift_actuator_write.publish(uswift_actuator_out)
    rospy.loginfo('Toggling the uSwift actuator')



# joystick callback specific to the spacenav
def spacenav_joy_callback(joy):
    global actuator_write_callback
    global control_roomba
    global spacenav_b0_pressed
    global spacenav_b1_pressed

    # rospy.loginfo('In the spacenav_twist callback')
    # rospy.loginfo('The first button is: %i', int(joy.buttons[0]))

    # in the future, we'll rely more on this basic type,
    # but for now, we're just using the buttons.
    if joy.buttons[0] and not spacenav_b0_pressed:
        # flip control between the roomba and the arm
        control_roomba = not control_roomba
        spacenav_b0_pressed = True
        if control_roomba:
            rospy.loginfo('Changing control to Roomba.')
        else:
            rospy.loginfo('Changing control to uArm.')

    if joy.buttons[1] and not spacenav_b1_pressed and not control_roomba:
        # toggle the actuator
        spacenav_b1_pressed = True
        actuator_write_callback(Bool(not uswift_actuator_out.data))

    if not joy.buttons[0] and spacenav_b0_pressed:
        # might have to create a threshold for bouncing
        spacenav_b0_pressed = False

    if not joy.buttons[1] and spacenav_b1_pressed:
        # might have to create a threshold for bouncing
        spacenav_b1_pressed = False



# spacenav twist callback
def spacenav_twist_callback(twist):
    global roomba_twist_write
    global cmd_vel
    global roomba_vector_scale
    global roomba_angular_scale
    global send_vector_roomba

    if control_roomba and send_vector_roomba:
        send_vector_roomba = False
        cmd_vel.linear.x = roomba_vector_scale * twist.linear.x
        cmd_vel.linear.y = roomba_vector_scale * twist.linear.y
        cmd_vel.linear.z = roomba_vector_scale * twist.linear.z
        cmd_vel.angular.x = roomba_angular_scale * twist.angular.x
        cmd_vel.angular.y = roomba_angular_scale * twist.angular.y
        cmd_vel.angular.z = roomba_angular_scale * twist.angular.z
        
        roomba_twist_write.publish(cmd_vel)
        # rospy.loginfo('Publishing a cmd_vel')
    elif not control_roomba:
        # this is the equivalent of sending a uarm message
        keyboard_teleop_callback(twist)



# generic joystick and controller callback
def joystick_callback(joy):
    global roomba_vector_scale
    global roomba_angular_scale
    global uswift_vector_scale
    global roomba_twist_write
    global uswift_vector_write
    global actuator_write_callback
    global uswift_vector_out
    global cmd_vel
    global xbox_a_pressed
    global send_vector_roomba

    # we may have to filter for low-amplitude signals
    # if the joy library doesn't have a setting for that already
    cmd_vel.linear.x = roomba_vector_scale * joy.axes[1]
    cmd_vel.angular.x = roomba_angular_scale * joy.axes[0] # eh one of these will work
    cmd_vel.angular.y = roomba_angular_scale * joy.axes[0]
    cmd_vel.angular.z = roomba_angular_scale * joy.axes[0]
    
    uswift_vector_out.x = uswift_vector_scale * joy.axes[4]
    uswift_vector_out.y = uswift_vector_scale * joy.axes[3]
    uswift_vector_out.z = uswift_vector_scale * (joy.axes[2] - joy.axes[5])  # not sure how I want to do this

    if joy.buttons[0] and not xbox_a_pressed:
        # toggle the actuator
        xbox_a_pressed = True
        actuator_write_callback(Bool(not uswift_actuator_out.data))

    if not joy.buttons[0] and xbox_a_pressed:
        # might have to create a threshold for bouncing
        xbox_a_pressed = False

    # rospy.loginfo('Publishing a cmd_vel')
    if send_vector_roomba:
        send_vector_roomba = False
        roomba_twist_write.publish(cmd_vel)
    # rospy.loginfo('Publishing a uSwift vector')
    uswift_vector_write.publish(uswift_vector_out)



# update rosparams
# This method is called every 500 milliseconds
def update_rosparams(event):
    global uswift_vector_scale
    global roomba_vector_scale
    global roomba_angular_scale

    # rospy.loginfo('updating rosparams')
    if rospy.has_param('/pnr_core/roomba_vector_scale'):
        roomba_vector_scale = rospy.get_param('/pnr_core/roomba_vector_scale')
    if rospy.has_param('/pnr_core/roomba_angular_scale'):
        roomba_angular_scale = rospy.get_param('/pnr_core/roomba_vector_scale')
    if rospy.has_param('/pnr_core/uswift_vector_scale'):
        uswift_vector_scale = rospy.get_param('/pnr_core/uswift_vector_scale')


# allow new vectors to be sent to the Roomba
def open_roomba_vector(event):
    global send_vector_roomba
    send_vector_roomba = True


#
#
# pnr_core
# 
def pnr_core():
    global uswift_vector_write
    global uswift_actuator_write
    global roomba_twist_write
    global uswift_vector_scale
    global roomba_vector_scale
    global roomba_angular_scale

    ## init rospy node 'pnr_core'
    rospy.init_node('pnr_core')

    rospy.loginfo('Initializing pnr_core in namespace %s',
        rospy.get_namespace())


    ## Publishers (initialization)
    uswift_vector_write = rospy.Publisher(
        '/pnr_swiftpro/vector_write_t', 
        Vector3, 
        queue_size=1)

    uswift_actuator_write = rospy.Publisher(
        '/pnr_swiftpro/actuator_write_t', 
        Bool, 
        queue_size=1)

    roomba_twist_write = rospy.Publisher(
        '/cmd_vel_t', 
        Twist, 
        queue_size=1)


    ## Subscribers
    rospy.Subscriber(
        '/teleop_keyboard/twist', 
        Twist, 
        keyboard_teleop_callback)

    rospy.Subscriber(
        '/spacenav/joy',
        Joy,
        spacenav_joy_callback)

    rospy.Subscriber(
        '/spacenav/twist',
        Twist,
        spacenav_twist_callback)

    rospy.Subscriber(
        '/joy',
        Joy,
        joystick_callback)


    ## test for parameters
    #
    # NB: For some reason, rospy always starts in the '/' namespace. 
    # Therefore, all resource referencing must be done with the absolute
    # resource path. (i.e. '/pnr_base/thing' instead of just 'thing')
    if not rospy.has_param('/pnr_core/uswift_vector_scale'):
        rospy.logwarn('rosparam "/pnr_core/uswift_vector_scale" not found.')
        rospy.logwarn(
            'Defaulting to /pnr_core/uswift_vector_scale = %.2f.', 
            uswift_vector_scale)
        rospy.set_param('/pnr_core/uswift_vector_scale', uswift_vector_scale)
    else:
        uswift_vector_scale = rospy.get_param('/pnr_core/uswift_vector_scale')

    if not rospy.has_param('/pnr_core/roomba_vector_scale'):
        rospy.logwarn('rosparam "/pnr_core/roomba_vector_scale" not found.')
        rospy.logwarn(
            'Defaulting to /pnr_core/roomba_vector_scale = %.2f.',
            roomba_vector_scale)
        rospy.set_param('/pnr_core/roomba_vector_scale', roomba_vector_scale)
    else:
        roomba_vector_scale = rospy.get_param('/pnr_core/roomba_vector_scale')

    if not rospy.has_param('/pnr_core/roomba_angular_scale'):
        rospy.logwarn('rosparam "/pnr_core/roomba_angular_scale" not found.')
        rospy.logwarn(
            'Defaulting to /pnr_core/roomba_angular_scale = %.2f.', 
            roomba_angular_scale)
        rospy.set_param('/pnr_core/roomba_angular_scale', roomba_angular_scale)
    else:
        roomba_angular_scale = rospy.get_param('/pnr_core/roomba_angular_scale')


    ## Main program loop
    while not rospy.is_shutdown():
        # update rosparams every 1000 ms
        rospy.loginfo('Updating rosparams every %g seconds...',
            ROSPARAM_UPDATE_PERIOD)
        rospy.Timer(rospy.Duration(ROSPARAM_UPDATE_PERIOD),
            update_rosparams)

        rospy.loginfo('Sending a Roomba vector every %g seconds...',
            0.2)
        rospy.Timer(rospy.Duration(0.2), open_roomba_vector) # hard-code for now...

        rospy.loginfo('pnr_core -- Waiting for callbacks...')
        # callbacks run automagically so long as this node is still alive
        rospy.spin()


if __name__ == '__main__':
    try:
        pnr_core()
    except rospy.ROSInterruptException:
        pass

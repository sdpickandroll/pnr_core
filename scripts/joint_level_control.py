#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Float64
# from std_msgs.msg import String
# from std_msgs.msg import Empty

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist


## Publishers 
# (we put these declarations here to make them global)
joint0_write = 0
joint1_write = 0
joint2_write = 0
joint3_write = 0
uswift_actuator_write = 0
roomba_twist_write = 0

roomba_vector_scale = 1.0
roomba_angular_scale = 1.0
uswift_degree_scale = 3.0


## Assorted global variables

# joint number --
# 0 => base
# 1 => shoulder
# 2 => elbow
# 3 => hand
# 4 => roomba forward/backward
# 5 => roomba right/left
joint = 0


# roomba flood protection
send_vector_roomba = True
flood_vector_roomba_thresh = 0.2

# uswift vector
uswift_value_out = Float64()

# uswift actuator
uswift_actuator_out = Bool(False)

# roomba cmd_vel
cmd_vel = Twist()

# spacenav button globals
spacenav_b0_pressed = False
spacenav_b1_pressed = False


## Callbacks

# teleop callback
# does nothing fancy; just forwards the sparse messages to the uSwift
def keyboard_teleop_callback(twist):
    global uswift_value_out
    global joint0_write
    global joint1_write
    global joint2_write
    global joint3_write

    uswift_value_out = Float64(twist.linear.y * uswift_degree_scale)

    rospy.loginfo('Publishing a uSwift vector to joint ' + joint)
    if joint == 0:
        joint0_write.publish(uswift_value_out)
    if joint == 1:
        joint1_write.publish(uswift_value_out)
    if joint == 2:
        joint2_write.publish(uswift_value_out)
    if joint == 3:
        joint3_write.publish(uswift_value_out)


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
    global spacenav_b0_pressed
    global spacenav_b1_pressed

    # rospy.loginfo('In the spacenav_twist callback')
    # rospy.loginfo('The first button is: %i', int(joy.buttons[0]))

    # in the future, we'll rely more on this basic type,
    # but for now, we're just using the buttons.
    if joy.buttons[0] and not spacenav_b0_pressed:
        spacenav_b0_pressed = True
        joint = 0 if joint >= 5 else joint + 1
        rospy.loginfo('Switching to joint ' + joint)

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

    if joint == 4 and send_vector_roomba:
        send_vector_roomba = False
        # TODO: Not sure if that's the right axis
        cmd_vel.linear.x = roomba_vector_scale * twist.linear.x
        roomba_twist_write.publish(cmd_vel)
        rospy.logdebug('Publishing a cmd_vel')
    elif joint == 5 and send_vector_roomba:
        send_vector_roomba = False
        # TODO: Not sure if that's the right axis
        cmd_vel.angular.z = roomba_angular_scale * twist.angular.z
        roomba_twist_write.publish(cmd_vel)
        rospy.logdebug('Publishing a cmd_vel')
    else:
        # this is the equivalent of sending a uarm message
        keyboard_teleop_callback(twist)


# update rosparams
# This method is called every 500 milliseconds
def update_rosparams(event):
    global roomba_vector_scale
    global roomba_angular_scale

    # rospy.loginfo('updating rosparams')
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
    global joint0_write
    global joint1_write
    global joint2_write
    global joint3_write
    global uswift_actuator_write
    global roomba_twist_write
    global roomba_vector_scale
    global roomba_angular_scale

    ## init rospy node 'pnr_core'
    rospy.init_node('pnr_core')

    rospy.loginfo('Initializing pnr_core in namespace %s',
        rospy.get_namespace())


    ## Publishers (initialization)
    joint0_write = rospy.Publisher(
        '/pnr_swiftpro/joint0_write',
        Float64,
        queue_size=1)
    joint1_write = rospy.Publisher(
        '/pnr_swiftpro/joint1_write',
        Float64,
        queue_size=1)
    joint2_write = rospy.Publisher(
        '/pnr_swiftpro/joint2_write',
        Float64,
        queue_size=1)
    joint3_write = rospy.Publisher(
        '/pnr_swiftpro/joint3_write',
        Float64,
        queue_size=1)

    uswift_actuator_write = rospy.Publisher(
        '/pnr_swiftpro/actuator_write', 
        Bool, 
        queue_size=1)

    roomba_twist_write = rospy.Publisher(
        '/cmd_vel', 
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
        # callbacks are called so long as this node is still alive
        rospy.spin()


if __name__ == '__main__':
    try:
        pnr_core()
    except rospy.ROSInterruptException:
        pass

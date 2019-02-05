#!/usr/bin/env python

import rospy


from std_msgs.msg import Bool
# from std_msgs.msg import String
# from std_msgs.msg import Empty

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Joy


## Publishers 
# (we put these declarations here to make them global)
uswift_vector_write = 0
uswift_actuator_write = 0
roomba_twist_write = 0


## Assorted global variables

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



## Callbacks

def keyboard_teleop_callback(twist):
    global uswift_vector_write
    global uswift_vector_out
    global uswift_vector_scale

    # update rosparam uswift_vector_scale
    if rospy.has_param('uswift_vector_scale'):
        uswift_vector_scale = rospy.get_param('uswift_vector_scale')

    if update_uswift_vector and not control_roomba:
        uswift_vector_out.x = uswift_vector_scale * twist.linear.x
        uswift_vector_out.y = uswift_vector_scale * twist.linear.y
        uswift_vector_out.z = uswift_vector_scale * twist.linear.z

        uswift_vector_write.publish(uswift_vector_out)
        rospy.loginfo('Publishing a uSwift vector')


def actuator_write_callback(state):
    global uswift_actuator_write
    global uswift_actuator_out

    uswift_actuator_out = state

    uswift_actuator_write.publish(uswift_actuator_out)
    rospy.loginfo('Toggling the uSwift actuator')


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
        uswift_actuator_out.data = not uswift_actuator_out.data
        rospy.loginfo('Toggling actuator.')

    if not joy.buttons[0] and spacenav_b0_pressed:
        # might have to create a threshold for bouncing
        spacenav_b0_pressed = False

    if not joy.buttons[1] and spacenav_b1_pressed:
        # might have to create a threshold for bouncing
        spacenav_b1_pressed = False


def spacenav_twist_callback(twist):
    global roomba_twist_write
    global cmd_vel
    global roomba_vector_scale
    global roomba_angular_scale

    if control_roomba:
        if rospy.has_param('roomba_vector_scale'):
            roomba_vector_scale = rospy.get_param('roomba_vector_scale')

        if rospy.has_param('roomba_angular_scale'):
            roomba_angular_scale = rospy.get_param('roomba_angular_scale')

        cmd_vel.linear.x = roomba_vector_scale * twist.linear.x
        cmd_vel.linear.y = roomba_vector_scale * twist.linear.y
        cmd_vel.linear.z = roomba_vector_scale * twist.linear.z
        cmd_vel.angular.x = roomba_angular_scale * twist.angular.x
        cmd_vel.angular.y = roomba_angular_scale * twist.angular.y
        cmd_vel.angular.z = roomba_angular_scale * twist.angular.z
        
        roomba_twist_write.publish(cmd_vel)
        rospy.loginfo('Publishing a cmd_vel')

    else:
        # this is the equivalent of sending a uarm message
        keyboard_teleop_callback(twist)


## Parameters
# /pnr_core/uswift_vector_scale
uswift_vector_scale = 1.0

# /pnr_core/roomba_vector_scale
roomba_vector_scale = 1.0

# /pnr_core/roomba_angular_scale
roomba_angular_scale = 0.5


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
        '/pnr_swiftpro/vector_write', 
        Vector3, 
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
        uswift_vector_scale = rospy.get_param('uswift_vector_scale')

    if not rospy.has_param('/pnr_core/roomba_vector_scale'):
        rospy.logwarn('rosparam "/pnr_core/roomba_vector_scale" not found.')
        rospy.logwarn(
            'Defaulting to /pnr_core/roomba_vector_scale = %.2f.',
            roomba_vector_scale)
        rospy.set_param('/pnr_core/roomba_vector_scale', roomba_vector_scale)
    else:
        roomba_vector_scale = rospy.get_param('roomba_vector_scale')

    if not rospy.has_param('/pnr_core/roomba_angular_scale'):
        rospy.logwarn('rosparam "/pnr_core/roomba_angular_scale" not found.')
        rospy.logwarn(
            'Defaulting to /pnr_core/roomba_angular_scale = %.2f.', 
            roomba_angular_scale)
        rospy.set_param('/pnr_core/roomba_angular_scale', roomba_angular_scale)
    else:
        roomba_angular_scale = rospy.get_param('roomba_angular_scale')


    ## Main program loop
    while not rospy.is_shutdown():
        rospy.loginfo('pnr_core -- Waiting for callbacks...')
        # callbacks run synchronously so long as this node is still alive
        rospy.spin()


if __name__ == '__main__':
    try:
        pnr_core()
    except rospy.ROSInterruptException:
        pass

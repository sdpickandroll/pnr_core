#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
# from std_msgs.msg import String
# from std_msgs.msg import Empty

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv
import imutils

import stitcher


# Publishers
output_panorama = 0

## Callbacks

# 
def


# update rosparams
# This method is called every x milliseconds
def update_rosparams(event):

    # rospy.loginfo('updating rosparams')
    if rospy.has_param('/pnr_core/roomba_vector_scale'):
        roomba_vector_scale = rospy.get_param('/pnr_core/roomba_vector_scale')
    if rospy.has_param('/pnr_core/roomba_angular_scale'):
        roomba_angular_scale = rospy.get_param('/pnr_core/roomba_vector_scale')
    if rospy.has_param('/pnr_core/uswift_vector_scale'):
        uswift_vector_scale = rospy.get_param('/pnr_core/uswift_vector_scale')



#
#
# stitcher
#
def stitcher():


    ## init rospy node 'pnr_stitcher'
    rospy.init_node('pnr_stitcher')

    ## Publishers (initialization)
    output_panorama = rospy.Publisher(
        '/pnr_swiftpro/front_panorama',
        Image,
        queue_size=1)


    ## Subscribers
    rospy.Subscriber(
        '/teleop_keyboard/twist',
        Twist,
        keyboard_teleop_callback)

    rospy.Subscriber(
        '/',
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
        # callbacks run synchronously so long as this node is still alive
        rospy.spin()


if __name__ == '__main__':
    try:
        pnr_core()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
# from std_msgs.msg import String
from std_msgs.msg import Empty

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Joy



## Parameters
# /pnr_core/uswift_vector_scale
uswift_vector_scale = 25.0

# /pnr_core/roomba_vector_scale
roomba_vector_scale = 0.2

# /pnr_core/roomba_angular_scale
roomba_angular_scale = 1.5

# update period (seconds)
ROSPARAM_UPDATE_PERIOD = 2

# vector publish period (seconds)
# (can be small since we throttle the message anyway)
VECTOR_PUBLISH_PERIOD = 0.05


## Publishers
# (we put these declarations here to make them global)
uswift_vector_write = 0
uswift_actuator_write = 0
roomba_twist_write = 0
uswift_home = 0


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

# quadstick globals
# indices are for the quadstick's first (i.e. leftmost) config ONLY
# yes, these are a mess. best to keep a list on hand.
quadstick_r_pf = False   # 7
quadstick_r_sp = False   # 5
quadstick_m_pf = False   # 2
quadstick_m_sp = False   # 0
quadstick_l_pf = False   # 6
quadstick_l_sp = False   # 4
quadstick_lm_sp = False  # 10
quadstick_rm_pf = False  # 3
quadstick_rm_sp = False  # 11
quadstick_lip = False    # 1

# xbox controller globals
xbox_a_pressed = False       # 0
xbox_b_pressed = False       # 1
xbox_x_pressed = False       # 2
xbox_y_pressed = False       # 3
xbox_lb_pressed = False      # 4
xbox_rb_pressed = False      # 5
xbox_back_pressed = False    # 6
xbox_start_pressed = False   # 7
xbox_power_pressed = False   # 8
xbox_lsb_pressed = False     # 9
xbox_rsb_pressed = False     # 10
xbox_d_up_pressed = False    # axes[-1] == 1.0
xbox_d_down_pressed = False  # axes[-1] == -1.0
xbox_d_left_pressed = False  # axes[-2] == 1.0
xbox_d_right_pressed = False # axes[-2] == -1.0


## Callbacks

# teleop calllback
# does nothing fancy; just forwards the sparse messages to the uSwift
def keyboard_teleop_callback(twist):
    global uswift_vector_write
    global uswift_vector_out

    # rospy.loginfo('Publishing a uSwift vector')

    uswift_vector_out.x = uswift_vector_scale * twist.linear.x
    uswift_vector_out.y = uswift_vector_scale * twist.linear.y
    uswift_vector_out.z = uswift_vector_scale * twist.linear.z


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


# provisional global for the spacenav_twist_callback
received_viable = True

# spacenav twist callback
def spacenav_twist_callback(twist):
    global roomba_twist_write
    global uswift_vector_out
    global cmd_vel
    global received_viable

    # test to make sure we're getting input. if not, ignore.
    if abs(twist.linear.x) + abs(twist.linear.y) + abs(twist.linear.y)\
        + abs(twist.angular.x) + abs(twist.angular.y)\
        + abs(twist.angular.z)  > 0.1:
        if control_roomba:
            cmd_vel.linear.x = roomba_vector_scale * twist.linear.x
            cmd_vel.angular.z = roomba_angular_scale * twist.angular.z

        elif not control_roomba:
            # this is the equivalent of sending a uarm message
            keyboard_teleop_callback(twist)
    elif received_viable:  # set everything to 0 if last message received
                           # was good but this one is bad
        cmd_vel.linear.x = 0
        cmd_vel.linear.z = 0
        uswift_vector_out = Vector3()
        received_viable = False
        



# xbox joystick callback
def xbox_callback(joy):
    global uswift_vector_scale
    global roomba_twist_write
    global uswift_vector_write
    global actuator_write_callback
    global uswift_vector_out
    global cmd_vel
    global xbox_a_pressed
    global xbox_b_pressed
    global xbox_back_pressed
    global xbox_start_pressed
    global xbox_d_up_pressed
    global xbox_d_down_pressed
    global xbox_d_left_pressed
    global xbox_d_right_pressed

    # set up the axes
    cmd_vel.linear.x = roomba_vector_scale * joy.axes[1]
    cmd_vel.angular.z = roomba_angular_scale * joy.axes[0]
    uswift_vector_out.x = uswift_vector_scale * joy.axes[4]
    uswift_vector_out.y = uswift_vector_scale * joy.axes[3]
    
    uswift_vector_out.z = 0.25 * uswift_vector_scale \
                          * (joy.axes[2] - joy.axes[5])

    if joy.buttons[0] and not xbox_a_pressed:
        # toggle the actuator
        xbox_a_pressed = True
        actuator_write_callback(Bool(not uswift_actuator_out.data))
    elif xbox_a_pressed:
        # might have to create a threshold for bouncing
        xbox_a_pressed = False

    if joy.buttons[1] and not xbox_b_pressed:
        # home the uswift
        xbox_b_pressed = True
        uswift_home.publish(Empty())
    elif xbox_b_pressed:
        xbox_b_pressed = False

    # track the start and back buttons for the adaptive controller
    # (for some reason, the right analog nunchuck registers its two
    # buttons as the back and the home button)
    if joy.buttons[6] and not xbox_back_pressed:
        xbox_back_pressed = True
    elif xbox_back_pressed:
        xbox_back_pressed = False
    if joy.buttons[7] and not xbox_start_pressed:
        xbox_start_pressed = True
    elif xbox_start_pressed:
        xbox_start_pressed = False

    # add the factor for the adaptive controller
    uswift_vector_out.z += 0.25 * uswift_vector_scale \
                           * (xbox_back_pressed - xbox_start_pressed)

    if joy.axes[-1] == 1.0 and not xbox_d_up_pressed:
        xbox_d_up_pressed = True
    elif xbox_d_up_pressed:
        xbox_d_up_pressed = False
    if joy.axes[-1] == -1.0 and not xbox_d_down_pressed:
        xbox_d_down_pressed = True
    elif xbox_d_down_pressed:
        xbox_d_down_pressed = False
    if joy.axes[-2] == 1.0 and not xbox_d_left_pressed:
        xbox_d_left_pressed = True
    elif xbox_d_left_pressed:
        xbox_d_left_pressed = False
    if joy.axes[-2] == -1.0 and not xbox_d_right_pressed:
        xbox_d_right_pressed = True
    elif xbox_d_right_pressed:
        xbox_d_right_pressed = False

    # add some factor for the d-pad
    cmd_vel.linear.x += roomba_vector_scale * \
                        (xbox_d_up_pressed - xbox_d_down_pressed)
    cmd_vel.angular.z += roomba_angular_scale * \
                         (xbox_d_left_pressed - xbox_d_right_pressed)



def quadstick_callback(joy):
    global quadstick_r_pf   # 7
    global quadstick_r_sp   # 5
    global quadstick_m_pf   # 2
    global quadstick_m_sp   # 0
    global quadstick_l_pf   # 6
    global quadstick_l_sp   # 4
    global quadstick_lm_sp  # 10
    global quadstick_rm_pf  # 3
    global quadstick_rm_sp  # 11
    global quadstick_lip    # 1
    global roomba_twist_write
    global uswift_vector_write
    global actuator_write_callback
    global uswift_vector_out
    global cmd_vel
    
    # there are 10 buttons. 10. buttons.
    # not all of them are used in this program, but I included all of
    # them here because I'm not sure what's going to be used at the end
    # of the day
    
    if joy.buttons[0] and not quadstick_m_sp:
        quadstick_m_sp = True
    elif quadstick_m_sp:
        quadstick_m_sp = False
        
    if joy.buttons[1] and not quadstick_lip:
        quadstick_lip = True
    elif quadstick_lip:
        quadstick_lip = False

    if joy.buttons[2] and not quadstick_m_pf:
        quadstick_m_pf = True
    elif quadstick_m_pf:
        quadstick_m_pf = False

    # toggles the actuator
    if joy.buttons[3] and not quadstick_rm_pf:
        # toggle actuator
        actuator_write_callback(Bool(not uswift_actuator_out.data))
        quadstick_rm_pf = True
    elif quadstick_rm_pf:
        quadstick_rm_pf = False

    if joy.buttons[4] and not quadstick_l_sp:
        quadstick_l_sp = True
    elif quadstick_l_sp:
        quadstick_l_sp = False

    if joy.buttons[5] and not quadstick_r_sp:
        quadstick_r_sp = True
    elif quadstick_r_sp:
        quadstick_r_sp = False

    if joy.buttons[6] and not quadstick_l_pf:
        quadstick_l_pf = True
    elif quadstick_l_pf:
        quadstick_l_pf = False

    if joy.buttons[7] and not quadstick_r_pf:
        quadstick_r_pf = True
    elif quadstick_r_pf:
        quadstick_r_pf = False

    # homes the uswift
    if joy.buttons[10] and not quadstick_lm_sp:
        # send a home empty
        uswift_home.publish(Empty())
        quadstick_lm_sp = True
    elif quadstick_lm_sp:
        quadstick_lm_sp = False

    if joy.buttons[11] and not quadstick_rm_sp:
        quadstick_rm_sp = True
    elif quadstick_rm_sp:
        quadstick_rm_sp = False

    uvs = uswift_vector_scale
    rvs = roomba_vector_scale
    ras = roomba_angular_scale
        
    uswift_vector_out.x = uvs * joy.axes[1]
    uswift_vector_out.y = uvs * joy.axes[0]
    uswift_vector_out.z = 0.5 * uvs * (quadstick_r_pf - quadstick_r_sp)

    cmd_vel.linear.x = rvs * 0.5 * (quadstick_m_pf - quadstick_m_sp)
    cmd_vel.angular.z = ras * 0.5 * (quadstick_l_sp - quadstick_l_pf)



# publish all controller-dependent messages
# called every VECTOR_PUBLISH_PERIOD seconds
#
# each of these vectors is calculated either in the xbox_callback method
# or the quadstick_callback method, whichever was called last
def publish(event):
    global roomba_twist_write
    global uswift_vector_write
    global cmd_vel
    global uswift_vector_out
    
    roomba_twist_write.publish(cmd_vel)
    # rospy.loginfo('Publishing a uSwift vector')
    uswift_vector_write.publish(uswift_vector_out)



# update rosparams
# called every ROSPARAM_UPDATE_PERIOD seconds
def update_rosparams(event):
    global uswift_vector_scale
    global roomba_vector_scale
    global roomba_angular_scale

    # rospy.loginfo('updating rosparams')
    if rospy.has_param('/pnr_core/roomba_vector_scale'):
        roomba_vector_scale = rospy.get_param('/pnr_core/roomba_vector_scale')
    if rospy.has_param('/pnr_core/roomba_angular_scale'):
        roomba_angular_scale = rospy.get_param('/pnr_core/roomba_angular_scale')
    if rospy.has_param('/pnr_core/uswift_vector_scale'):
        uswift_vector_scale = rospy.get_param('/pnr_core/uswift_vector_scale')



#
#
# pnr_core
#
def pnr_core():
    global uswift_vector_write
    global uswift_actuator_write
    global roomba_twist_write
    global uswift_home
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

    # NB: this one does not have a '_t' at the end because it doesn't
    # get throttled.
    uswift_actuator_write = rospy.Publisher(
        '/pnr_swiftpro/actuator_write',
        Bool,
        queue_size=1)

    roomba_twist_write = rospy.Publisher(
        '/cmd_vel_t',
        Twist,
        queue_size=1)

    uswift_home = rospy.Publisher(
        '/pnr_swiftpro/home',
        Empty,
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
        '/quadstick/joy',
        Joy,
        quadstick_callback)
    
    rospy.Subscriber(
        '/xbox_adaptive/joy',
        Joy,
        xbox_callback)

    rospy.Subscriber(
        '/xbox/joy',
        Joy,
        xbox_callback)


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
        rospy.loginfo('Set to update rosparams every %g seconds.',
            ROSPARAM_UPDATE_PERIOD)
        rospy.Timer(rospy.Duration(ROSPARAM_UPDATE_PERIOD),
            update_rosparams)
        rospy.loginfo('Set to publish vectors every %g seconds.',
            VECTOR_PUBLISH_PERIOD)
        rospy.Timer(rospy.Duration(VECTOR_PUBLISH_PERIOD),
            publish)

        rospy.loginfo('pnr_core -- Waiting for callbacks...')
        # callbacks run automagically so long as this node is still alive
        rospy.spin()


if __name__ == '__main__':
    try:
        pnr_core()
    except rospy.ROSInterruptException:
        pass

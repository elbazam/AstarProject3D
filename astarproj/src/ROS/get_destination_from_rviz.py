#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion


from geometry_msgs.msg import PoseStamped



def get_goal():

    PS_name = '/move_base_simple/goal'
    
    msg = rospy.wait_for_message(PS_name,PoseStamped)

    x , y = msg.pose.position.x , msg.pose.position.y
    q = msg.pose.orientation
    q_l = [q.x,q.y,q.z,q.w]
    (_,_,yaw) = euler_from_quaternion(q_l)

    return x,y,yaw
    
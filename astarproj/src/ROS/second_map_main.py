#!/usr/bin/env python


import rospy
import numpy as np
import cv2

import matplotlib.pyplot as plt

from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped , Twist
from nav_msgs.msg import Odometry

from Astarcodes import astar , reconstract_path , world2maplocation , Save_A_star_Solution , valid_point , save_configuration_as_img
from get_map import get_occupancy_data , build_A_star_required_map , reconstract_map
from get_destination_from_rviz import get_goal


def publish_vel(front , rad , pub):

    T = Twist()
    T.linear.x = front
    T.linear.y = 0
    T.linear.z = 0
    T.angular.z = rad
    T.angular.y = 0
    T.angular.x = 0
    for _ in range(5): pub.publish(T)


def callback(msg):

    global pub , cond , map_building_cond , a , b , path , ros_path , idx , res , obs , orig , x_start , y_start , old_d
    pos = msg.pose.pose.position
    q = msg.pose.pose.orientation
    xr , yr = pos.x , pos.y
    q_l = [q.x,q.y,q.z,q.w]
    (_,_,thr) = euler_from_quaternion(q_l)

    if cond:
        idx = 0
        if map_building_cond:
            x_start = -3.0
            y_start = 1.0
            
            map_building_cond = False
            print('Build obsticles data from occupancy grid msg...')
            m , res = get_occupancy_data()
            orig = np.array([x_start , y_start , 0])
            newmap = reconstract_map(m)
            
            obs = build_A_star_required_map(m ,a , b , res , theta_inc = np.pi/18)
            save_configuration_as_img(obs[:,:,0] , name = 'Configuration 0 2.png' , crop = False)
            save_configuration_as_img(obs[:,:,3] , name = 'Configuration 30 2.png' , crop = False)
            save_configuration_as_img(obs[:,:,6] , name = 'Configuration 60 2.png' , crop = False)
            print('Finished generating obsticles matrix data')
        else:
            m , _ = get_occupancy_data()
            newmap = reconstract_map(m)
        while True:
            print('Please insert a goal using rviz')
            x,y,th = get_goal()
            print(round(x,3),round(y,3),round(th,3))
            point = np.array([x,y,th])
            goal = world2maplocation(obs , point , res , orig)
            print(goal[0],goal[1])
            if valid_point(obs , goal): break
            else: print('Bad point, try again')
        point = np.array([xr-x_start,yr-y_start,thr])
        start = world2maplocation(obs , point , res , orig)
        print('If begining is occupied it will be written 1: ' , obs[start[0],start[1],start[2]])
        path , visited = astar(start,goal,obs)
        l = len(path)
        
        if l:
            ros_path = reconstract_path(obs , path , res , orig)
        Save_A_star_Solution(newmap , start , goal ,
                            path, visited ,
                            name = 'A_star_solution_2.png',
                            crop = False)
        print('Saved the solution')
        cond = False
    
    l = len(path)
    # ======================= Controller / local planner =============================
    if idx < l:
        while idx < l-2:
            if path[idx][2] == path[idx + 1][2]:
                idx = idx + 1
                continue
            else:
                break
        xg  = ros_path[idx][0]
        yg  = ros_path[idx][1]
        thg = ros_path[idx][2]

        x_dis = xg - xr
        y_dis = yg - yr
        
        if thr < 0:
            thr = thr + 2*np.pi
        th_to_target = np.arctan2(y_dis , x_dis)

        th_req = thg - thr
        d = np.sqrt(x_dis ** 2 + y_dis**2)

        threshold = (np.pi / 18)*1.0
        thre_dist = res * 2

        if d > thre_dist:
            
            th_needed = np.arctan2(y_dis , x_dis)
            if th_needed < 0:
                th_needed = 2*np.pi + th_needed
            th_error = th_needed - thr
            if np.absolute(th_error) > threshold:
                if np.absolute(th_error) > np.pi:
                    th_error = -th_error
                publish_vel(0,np.sign(th_error)*threshold/2,pub)
            else:
                publish_vel(0.1,0,pub)
        elif np.abs(th_req) > threshold:
                if np.abs(th_req) > np.pi:
                    th_req = -th_req
                
                publish_vel(0,np.sign(th_req)*threshold/2,pub)
        else:
            
            idx = idx + 1
            if idx < l:
                print('Moving to the next point:')
                print( round(ros_path[idx][0],3) ,  round(ros_path[idx][1],3) ,round((180/np.pi)*ros_path[idx][2],3))
                print("Point index: [" , idx , "/" ,l , "]")
            
        
        
    else:
        publish_vel(0,0,pub)
        cond = True
        if l > 0:
            print('Reached the goal')
        else:
            print('Could not reach the goal')
    
        


def code():

    global pub , cond , map_building_cond , a , b , path  , idx , old_d
    cond = True
    map_building_cond = True
    path = []
    idx = 0
    a = 6
    b = 6
    old_d = 10
    rospy.init_node('main_code_node', anonymous=True)

    pub = rospy.Publisher('/cmd_vel' , Twist , queue_size = 1)

    rospy.Subscriber('/odom' , Odometry , callback)

    rospy.spin()

if __name__ == "__main__":

    try: code()
    except rospy.ROSInterruptException: pass
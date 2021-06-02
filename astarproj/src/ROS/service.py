#!/usr/bin/env python

import rospy
import numpy as np
import time
import cv2

from global_path_seeker.api import UISS_map
from matrix_check import is_different , new_list

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger , TriggerResponse

def publish_PoseStampe(loc,pub,cond = 0):

    P = PoseStamped()
    P.header.stamp = rospy.Time.now()
    
    P.pose.position.x = loc[0]
    P.pose.position.y = loc[1]
    P.pose.position.z = cond
    for _ in range(5):
        pub.publish(P)

def new_reconstract_map(m):
    
    m[m == -1] = 50
    m = 250 - 125*(np.array(m/50,dtype=np.uint8))
    return m


class exp:

    def __init__(self):
        '''Experiment class for trying UISS global planning method'''
        rospy.init_node('path_sender',anonymous=True)
        self.new_service = rospy.Service('/path_seeking_request',Trigger,self.generate_next_point)
        self.p_pub = rospy.Publisher('/next_goal' , PoseStamped , queue_size=1)
        self.path = []
        self._listener()
        
        self.map = np.zeros((20,20))
        self.resolution = 0.01
        self.UISS = UISS_map(self.map, start = [0,0],resolution = self.resolution,s=3 , MDW = 3)
        self.action = True
        self.img_show = False
        self._nextpoints()
        
                     
    def generate_next_point(self,req):
        '''Service response'''
        if self.path:
            self.path.pop(0)
            self.path = new_list(self.path , self.robot_location)
            if self.path:
                
                publish_PoseStampe(self.path[0], self.p_pub)
                return TriggerResponse(success=True , message = "Request has been made, sending next subgoal")
            else:
                print('Please input a new destination.')
                self.path = []
                self._nextpoints()
                if self.path:
                    self.path = new_list(self.path , self.robot_location)
                    publish_PoseStampe(self.path[0], self.p_pub)
                    return TriggerResponse(success=True , message = "New task has been recieved.")
                else:
                    
                    self.action = False
                    return TriggerResponse(success=False , message = "Finished the task, exit program.")
        else:
            print('Requiring a new goal')
            self.path = []
            self._nextpoints()
            if self.path:
                self.path.pop(0)
                self.path = new_list(self.path , self.robot_location)
                publish_PoseStampe(self.path[0], self.p_pub)
                return TriggerResponse(success=True , message = "New task has been recieved.")
            else:
                self.action = False 
                return TriggerResponse(success=False , message = "Finished the task, exit program.")

    def check_path(self):
        '''If path is not empty, pubish the path '''
        if self.path:
            self.path = new_list(self.path , self.robot_location , msg = True)
            self.p = self.path[0]
            if self.p == self.path[-1]: cond = 1
            else: cond = 0
            publish_PoseStampe(self.p, self.p_pub , cond)
            return True
        else:
            if self.action == False:
                return False
            else:
                return True
    
    def _odom_callback(self,msg):
        self.robot_location_msg = msg

    def _listener(self):
        '''Callback initialization '''
        rospy.wait_for_message('/mcl_pose',Odometry)
        rospy.Subscriber('/mcl_pose',Odometry,self._odom_callback)

    def _update_robot_location(self):
        '''Update the robot location'''
        self.robot_location = np.array([self.robot_location_msg.pose.pose.position.x,
                                    self.robot_location_msg.pose.pose.position.y])

    def _nextpoints(self):
        '''generate next goal and the path'''
        newmap , resolution , _ = is_different(self.map.shape)
        self.map = newmap
        self._update_robot_location()
        self.UISS.update(self.robot_location,newmap,resolution)
        
        cond = input('To insert new goal press 1. To finish inserting press 0: ')
        cond = int(cond)
        if cond > 0:
            while(1):
                x,y = input('Input new x y location: ').split()
                x = float(x)
                y = float(y)

                point = self.UISS.world2maplocation(np.array([x,y]))
                
                if self.UISS.set_goal(point):
                    self.action = True
                    t = time.time()
                    print('Path planning begins now.')
                    self.UISS.UISSPathPlanning()
                    if not self.UISS.Real_path:
                        print('Unreachable, insert new goal.')
                        continue
                    self.path = self.UISS.Real_path
                    print('Planning took ' , round(time.time() - t,3) , 'seconds')
                    self.img_show = True
                    
                    break
                
        else:
            self.action = False
            
            

e = exp()

while not rospy.is_shutdown():
    rospy.sleep(0.1)
    if not e.check_path(): break
    
    if e.img_show:
        new_img = new_reconstract_map(e.map)
        new_img = cv2.cvtColor(new_img,cv2.COLOR_GRAY2RGB)
        

        color = np.random.randint(255, size=3)
        color = tuple([int(color[0]),int(color[1]),int(color[2])])
        for i in range(len(e.UISS.path)-1):
            cv2.line(new_img , (e.UISS.path[i][1],e.UISS.path[i][0]) , (e.UISS.path[i+1][1], e.UISS.path[i+1][0]) , color , 3)
        e.img_show = False
    
    cv2.imshow('Path',new_img)
    cv2.waitKey(1)
    

def msg_on_shutdown():
    print('Goodbye until next time.')
    cv2.destroyAllWindows()

rospy.on_shutdown(msg_on_shutdown)






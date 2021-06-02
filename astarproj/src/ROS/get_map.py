#!/usr/bin/env python

import rospy
import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid



def get_occupancy_data():
    '''Return occupancy grid map as 2d matrix and the resolution of the map.
    return [Map , resolution]'''

    msg = rospy.wait_for_message('/map',OccupancyGrid)
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    data = np.array(msg.data,dtype=np.int8)
    return data.reshape((height,width)) , float(resolution)

def reconstract_map(m):
    
    m[m == -1] = 100
    m = 125*(np.array(m/50,dtype=np.uint8))
    return m
    
def matrix_size(a , b):
    '''a , b: car length and width in pixels'''
    siz = int(np.sqrt(a**2 + b**2) + 1)
    return siz

def _wheels_loc(a,b,theta , pixel_size = 0.1):
    '''Locations of wheels in pixel world '''
    
    Orig_fl = np.array([a/2,b/2])
    Orig_fr = np.array([a/2,-b/2])
    Orig_bl = np.array([-a/2,b/2])
    Orig_br = np.array([-a/2,-b/2])
    
    transf_mat = np.array([[np.cos(theta) , -np.sin(theta)],
                           [np.sin(theta) , np.cos(theta)]])
    
    fl = (np.dot(transf_mat , Orig_fl)).flatten()
    fr = (np.dot(transf_mat , Orig_fr)).flatten()
    bl = (np.dot(transf_mat , Orig_bl)).flatten()
    br = (np.dot(transf_mat , Orig_br)).flatten()
    
    loc = np.zeros((4,2))
    loc[0,0] , loc[0,1] = fl[0] , fl[1]
    
    loc[1,0] , loc[1,1] = bl[0] , bl[1]
    loc[2,0] , loc[2,1] = br[0] , br[1]
    loc[3,0] , loc[3,1] = fr[0] , fr[1]
    
    loc = np.array(loc,dtype = np.int8)
    
    return loc

def locations_set(a,b,theta_inc,pixel_size = 0.1):
    '''Generate set of locations depends on the car orientation'''
    N = int(2 * np.pi // theta_inc) - 1
    
    loc_set = np.zeros((4,2,N))

    for i in range(N):
        
        theta = i * theta_inc
        loc_set[:,:,i] = _wheels_loc(a,b,theta , pixel_size = pixel_size)
    
    return loc_set
    
def mat_set(a , b , loc_set):
    '''Generate the occupancy grid that the car is taking on the map'''
    size = matrix_size(a, b)
    Number_of_matrix = loc_set.shape[2]
    
    M = np.zeros((size,size,Number_of_matrix),dtype = np.uint8)
    
    car_loc = np.array([size/2 , size/2] , dtype = np.int8)
    
    for i in range(Number_of_matrix):
        L = np.zeros((size,size))
        loc_set[:,:,i] = loc_set[:,:,i] + car_loc
        
        cv2.fillPoly(L, pts =[np.array(loc_set[:,:,i],dtype = int)], color=(255))
        M[:,:,i] = L
        
    return M , car_loc

def configuration_space(M,car_loc,Occup):
    '''Generate the configuration space'''
    lim = M.shape[0]
    L1 = Occup.shape[0]
    L2 = Occup.shape[1]
    N = M.shape[2]
    
    obs = np.ones((L1,L2,N))
    
    for i in range(lim,L1 - lim):
        for j in range(lim,L2-lim):
            if Occup[i,j] > 0:
                obs[i,j,:] = 1
                continue
            ind_x = i - car_loc[0]
            ind_y = j - car_loc[1]
            check_smaller_space = Occup[ind_x:ind_x + lim , ind_y:ind_y + lim]
            
            for k in range(N):
                p = np.sum(check_smaller_space * M[:,:,k])
                if p == 0:
                    obs[i,j,k] = 0
    for j in range(N):
        rev = obs[:,:,j]
        obs[:,:,j] = rev.T
    
    return obs

def build_A_star_required_map(m ,a , b , res , theta_inc = np.pi/18):

    Occup = reconstract_map(m)
    loc_set = locations_set(a, b, theta_inc , pixel_size=res)
    M , car_loc = mat_set(a,b,loc_set)
    return configuration_space(M,car_loc,Occup)



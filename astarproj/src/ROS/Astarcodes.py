#!/usr/bin/env python

from heapq import heappush, heappop
import numpy as np
import cv2

def Set_of_actions_from_node(theta , theta_in_rad = (np.pi/180)*(theta*10)):
    '''Generate actions from a given node. 
    The actions: {Forward , Turn left , Turn right}'''

    movements = [
                    (np.round(np.cos(theta_in_rad)),np.round(np.sin(theta_in_rad)), 0, 1.),
                    (0 , 0, 1, 1),
                    (0 , 0, -1, 1)
                ]
    return movements


def distance(p, q):
    """Helper function to compute distance between two points."""
    return np.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2)


def astar(start, goal, obstacles):
    
    goal = [int(_) for _ in goal]
    dist = distance(start,goal) + 0.001
    # Front element consists of: (Total cost , actions cost to this node, this node, prev node)
    front = [ (dist,0.001, start, None) ]
    
    # In the beginning, no cell has been visited.
    extents = obstacles.shape
    visited = np.zeros(extents, dtype=np.float32)

    # Also, we use a dictionary to remember where we came from.
    came_from = {}

    # While front is not empty.
    while front:
        
        # Lowest cost next node is chosen and removedc from front.
        element = heappop(front)
        
        
        _, cost, pos, previous = element[0],element[1],\
                                            element[2],element[3]

        # Now it has been visited. Mark with cost.
        pos = [int(_) for _ in pos]
        
        movements = Set_of_actions_from_node(pos[2])
        # Now it is visited. Mark with cost.
        if visited[tuple(pos)] > 0: continue
        else: visited[tuple(pos)] = cost

        
        came_from[tuple(pos)] = previous
        
        # Also remember that we came from previous when we marked pos.

        # Check if the goal has been reached.
        if pos == goal:
            print('A* found a path!')
            break  # Finished!

        # Check all neighbors.
        for dx, dy, dth , deltacost in movements:
            loc = tuple(map(sum, zip(pos, tuple((dx,dy,dth)))))
            new_x , new_y , new_theta = loc[0] , loc[1] , loc[2]
            if new_x < 0 or new_x >= extents[0]:
                continue
            if new_y < 0 or new_y >= extents[1]:
                continue
            if new_theta == extents[2]:
                new_theta = 0
            if new_theta == -1:
                new_theta = extents[2] - 1
            # Add to front if: not visited before and no obstacle.
            new_pos = (int(new_x), int(new_y), int(new_theta))
            if visited[tuple(new_pos)] == 0 and obstacles[tuple(new_pos)] == 0:
                new_cost = cost + deltacost
                new_total_cost = distance(new_pos,goal) + new_cost
                heappush(front,(new_total_cost,new_cost,new_pos,pos))
            

    # Reconstruct path, starting from goal.

    path = []
    if pos == goal:  # If we reached the goal, unwind backwards.
        while pos:
            path.append(pos)
            pos = came_from[tuple(pos)]
        path.reverse()  # Reverse so that path is from start to goal.

    return (path, visited)


def reconstract_path(Occup , path , res , orig):
        '''Generate the path as real world coordinates'''
        
        map_center = np.array(np.array(Occup.shape)/2,dtype = np.int16)
        
        rec_path = []
        
        for p in path:
            x = (p[0] - map_center[0])*res + orig[0]
            y = (p[1] - map_center[1])*res + orig[1]
            th = (10*p[2]) * (np.pi / 180)
            rec_path.append([x,y,th])
        return rec_path
    
    
def world2maplocation(Occup , point , res , orig = np.array([0,0],dtype = int)):
    '''Transfer the request from real world to the map world '''
    

    x = Occup.shape[0]/2
    y = Occup.shape[1]/2
    th = 0
    map_center = np.array([x,y,th])

    new_point = point
    new_point = (new_point + map_center*res)/res
    new_point = np.array(new_point,dtype = int)
    
    if point[2] < 0:
        point[2] = point[2] + 2*np.pi
    new_point[2] = int(point[2] * (180/np.pi) / 10)
    if new_point[2] == 35:
        new_point[2] = int(0)

    return new_point

def valid_point(obs , point):
    '''True - unoccupied location ; False - occupied location '''
    if obs[point[0] , point[1] , point[2]] != 0: return False
    else: return True

def Save_A_star_Solution(Occup , start , goal , path, visited , name = 'A_star_solution.jpg' , crop = True):

    v = np.where(visited > 0)
    newmap = cv2.cvtColor(Occup,cv2.COLOR_GRAY2RGB)
    
    for i in range(v[0].shape[0]):
        cv2.circle(newmap, (v[0][i],v[1][i]), 1, (255,0,0), 1)
    for i in range(len(path)):
        cv2.circle(newmap, (path[i][0],path[i][1]), 1, (255,0,255), 1)
    cv2.circle(newmap, (start[0],start[1]), 6, (0,255,0), 1)
    cv2.circle(newmap, (goal[0],goal[1]), 6, (0,0,255), 1)
    if crop:
        crop_img = newmap[950:1150, 950:1150]
        crop_img = cv2.resize(crop_img , (1024,1024))


        cv2.imwrite(name , crop_img)
    else:
        cv2.imwrite(name , newmap)


def save_configuration_as_img(img , name = 'configuration' , crop = True):

    img = img * 255
    img = np.array(img , dtype = np.uint8)
    if crop:
        crop_img = img[950:1150, 950:1150]
        crop_img = cv2.resize(crop_img , (1024,1024))

        cv2.imwrite(name , crop_img)
    else:
        cv2.imwrite(name , img)

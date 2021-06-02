#!/usr/bin/env python

import rospy
import numpy as np
from scipy.io import savemat

from nav_msgs.msg import OccupancyGrid



def reshape_msg2map():
    msg = rospy.wait_for_message('/map',OccupancyGrid)
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    data = np.array(msg.data,dtype=np.int8)
    return data.reshape((height,width)) , resolution


if __name__ == "__main__":

    rospy.init_node('map_getting_node', anonymous=True)

    OccMap , res = _reshape_msg2map()
    print('Got the map')
    dic = {'map' : OccMap , 'resolution' : res}

    savemat('NewMap.mat' , dic)
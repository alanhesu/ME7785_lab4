#!/usr/bin/env python

import rospy
from greenbutter_navigate_to_goal.msg import ObjDetect
import numpy as np
from sensor_msgs.msg import LaserScan

def laser_callback(data):
    thmin = 2*np.pi - np.pi/2
    thmax = np.pi/2
    dist_thresh = 0.4
    obj_msg = ObjDetect()

    increment = data.angle_increment
    ranges = list(data.ranges)
    # set all zero values to something else
    # ranges[ranges <= 0.01] = np.inf
    ranges[:] = [np.inf if val == 0 else val for val in ranges]

    ind_min=int(np.round(thmin/ increment))
    ind_max=int(np.round(thmax/ increment))
    fov_range=ranges[ind_min:ind_max]

    min_dist = min(ranges)
    min_th_ind = ranges.index(min_dist)


    # Averaging LIDAR values
    if min_dist != np.inf:
        min_dist_arr = list(np.zeros(9))
        o = 0

        for i in range(min_th_ind-4,min_th_ind+5):
            if i > 359:
                i-= 360
            min_dist_arr[o] = ranges[i]
            o+=1

        min_dist_arr = [x for x in min_dist_arr if x !=np.inf]
        min_dist = np.mean(min_dist_arr)

    if (min_dist < 0.4):
        if (min_th_ind >= ind_min or min_th_ind <= ind_max):
            obj_msg.cone_detected = True
        else:
            obj_msg.cone_detected = False

        obj_msg.obj_detected = True
    else:
        obj_msg.obj_detected = False
    obj_msg.d1 = min_dist
    obj_msg.th1 = min_th_ind*increment
    if (obj_msg.th1 > np.pi):
        obj_msg.th1 -= 2*np.pi
    obj_pub.publish(obj_msg)

def find_object():
    global obj_pub
    rospy.init_node('detect_object')
    obj_pub = rospy.Publisher('/detected_obj', ObjDetect, queue_size=1)
    laser_sub = rospy.Subscriber("/scan", LaserScan, laser_callback)

    rospy.spin()

if __name__ == '__main__':
    find_object()

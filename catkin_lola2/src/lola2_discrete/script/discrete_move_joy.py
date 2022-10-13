#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 21 08:57:02 2022

@author: perseo
"""
import rospy
import time
import discrete_move2 as dmove
from sensor_msgs.msg import Joy


def callback(data):
    global semaforo, key
    lista = [0,1,2,3]
    if semaforo ==0:
        index = [i for i,e in enumerate(data.buttons) if e!=0]
        if index !=[] and index[0] in lista:
            semaforo = 1
            key = index[0]
            
if __name__ == '__main__':
    global semaforo, key
    semaforo = 1    
    r = dmove.MoveOdom()
    r.start_ros()
    rospy.Subscriber("/joy",Joy, callback)
    command = {0:r.move_backward,1:r.turn_right_b,2:r.turn_left_b,
                  3:r.move_straight}
    while r.odom_pose is None and not rospy.is_shutdown():
        time.sleep(0.1)
    semaforo = 0    
    while not rospy.is_shutdown():
        if semaforo == 1:
            command[key]()
            r.stop()
            semaforo = 0
        
        

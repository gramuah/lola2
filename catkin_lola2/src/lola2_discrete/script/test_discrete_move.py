#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jul  5 10:12:46 2022

@author: perseo
"""
import rospy
import time
import discrete_move2 as dmove

if __name__ == '__main__':
    r = dmove.MoveOdom()
    r.start_ros()
    while r.odom_pose is None and not rospy.is_shutdown():
        time.sleep(0.1)
    #ejemplo de secuencia de comandos
    r.move_straight()
    r.stop()
    r.turn_right_b()
    r.stop()
    #r.move_straight()
    r.turn_left_b()
    r.stop()
    
    r.move_backward()
    r.stop()
    time.sleep(0.5)
    r.turn_right_b()
    r.turn_right_b()
    r.stop()
    #r.move_straight()
    r.turn_left_b(2)
    r.stop()
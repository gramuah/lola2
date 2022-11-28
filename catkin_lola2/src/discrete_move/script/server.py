#!/usr/bin/env python3

import rospy
from ServerClass import Server

if __name__ == '__main__':

    rospy.init_node('server')
    Server()

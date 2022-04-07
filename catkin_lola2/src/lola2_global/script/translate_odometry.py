#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 21 05:42:02 2021

@author: perseo
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 16 10:20:03 2021

@author: perseo
"""
import rospy
from tf.transformations import euler_from_quaternion
from math import degrees
from lola2_global.msg import Odom2
from nav_msgs.msg import Odometry

def conversion(msg):
    odom2 = Odom2()
    odom2.x = msg.pose.pose.position.x
    odom2.y = msg.pose.pose.position.y
    q = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    a = euler_from_quaternion(q)
    odom2.angle = degrees(a[2]) 
    pub.publish(odom2)
    
if __name__=='__main__':
    global pub
    try:
        rospy.init_node('translate_odometry')
        pub = rospy.Publisher("odom2",Odom2, queue_size=1)
        rospy.Subscriber('odom',Odometry, conversion)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

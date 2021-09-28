#!/usr/bin/env python
#PROGRAMA TEST PRUEBA control_loop
import rospy
from geometry_msgs.msg import Twist
import time

class TestControl:
    def __init__(self):
        self.pub1 = rospy.Publisher("cmd_vel",Twist,queue_size=1)
        rospy.init_node("testcontrol")
        self.frecuencia_bucle = 10
        self.ciclos_cambio = 200
        
        
    def main(self):
        time.sleep(3)
        lista_v = [0.16,0.16,0.5,0.5]
        lista_w = [0,0,0.0,0.0,0.0]
        r = rospy.Rate(self.frecuencia_bucle)
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        while not rospy.is_shutdown():
            for v,w in zip(lista_v,lista_w):
                msg.linear.x,msg.angular.z = v,w
                for j in range(0,self.ciclos_cambio):
                    self.pub1.publish(msg)
                    rospy.loginfo("Enviado mensaje: V "+str(v) + " W " +str(w) )
                    r.sleep()
                
if __name__ == '__main__':
    f = TestControl()
    f.main()

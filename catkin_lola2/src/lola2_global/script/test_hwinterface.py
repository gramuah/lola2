#!/usr/bin/env python
#PROGRAMA TEST PRUEBA HWINTERFACE_SCRIPT_LOLA2
import rospy
from sensor_msgs.msg import JointState
import random

class TestHw:
    def __init__(self):
        self.publish_freq = 5
        self.maximos_ciclos_velocidad_parecida = 72
        self.contador_velocidades = self.maximos_ciclos_velocidad_parecida
        self.w_derecha_comando = 0
        self.w_izquierda_comando = 0
        self.start = 0
        self.cambio = 0
        
    def main(self):
        rospy.init_node('testinterface')
        rospy.loginfo("Iniciado nodo TEST HARD")
        self.data_pub = rospy.Publisher("cmd_wheel", JointState, queue_size=1)
        rospy.Subscriber("wheel_state", JointState, self.callback_velfeedback)
        r = rospy.Rate(self.publish_freq)
               
        while not rospy.is_shutdown():
            self.send_speed_command()
            r.sleep()
            
            
    def callback_velfeedback(self,msg):
        if(msg.name == ["RIGHT"]):
            w_derecha_recibida = msg.velocity[0]
            diferencia_derecha = self.w_derecha_comando - w_derecha_recibida
            rospy.loginfo("DD: %.2f WD: %.2f WDR: %.2f"% (diferencia_derecha,self.w_derecha_comando,w_derecha_recibida))
        else:
            w_izquierda_recibida = msg.velocity[0]
            diferencia_izquierda = (self.w_izquierda_comando - 
                                    w_izquierda_recibida)
            rospy.loginfo("DI: %.2f WI: %.2f WIR: %.2f"% (diferencia_izquierda,self.w_izquierda_comando,w_izquierda_recibida))
        
    def send_speed_command(self):         
        if(self.contador_velocidades == 
           self.maximos_ciclos_velocidad_parecida):
            if(self.cambio == 0):   
                self.w_derecha_comando = 1.5
                self.w_izquierda_comando = 1.5
                self.cambio = 1
            else:
                self.w_derecha_comando = 0
                self.w_izquierda_comando = 0
                self.cambio = 0
            self.contador_velocidades = 0
        else:
            self.contador_velocidades +=1
           
        
        data = JointState()
        rospy.loginfo("Enviado: "+str("%.2f"%self.w_derecha_comando)+"  "+str("%.2f"%self.w_izquierda_comando))
        data.name = ["Comando"]
        data.position = [0]
        data.velocity = [self.w_izquierda_comando,self.w_derecha_comando]
        data.effort = [0]
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = "base_link"
        self.data_pub.publish(data)
        
        
if __name__ == '__main__':
    foo = TestHw()
    foo.main()
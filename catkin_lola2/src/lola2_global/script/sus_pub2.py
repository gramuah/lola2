#!/usr/bin/env python

import serial, time
import struct
import rospy
from math import copysign, pi
from sensor_msgs.msg import JointState



class HwClass:
# Function: __init__
#
# Comments
# ----------
# Constructor of the class HW__CLASS
# Define system constants.
#
# Parameters
# ----------
#
# Returns
# -------
    def __init__(self):
        #constant parameters
        #TODO: Estos parametros deberian estar
        #      en un servidor de parametros y leerlos.
        self.pasos_vuelta= 356.3*2
        #self.pasos_vuelta= 1425.2
        self.left = 1
        self.right = 0
        self.kdato = 38.13
        self.twistTimeout = 5
        self.read_encoder_freq = 10
        self.encoderd = 0
        self.timerd = 0
        self.encoderi = 0
        self.timeri = 0
        self.time_enc_right_last = 0
        self.steps_enc_right_last = 0
        self.time_enc_left_last = 0
        self.steps_enc_left_last = 0
	    time.sleep(1.5)
	


       


# Function: main
#
# Comments
# ----------
# Set up the node, the publications and subsciptions.
#
# Parameters
# ----------
#
# Returns
# -------
    def main(self):
        rospy.init_node('hwinterface')
        self.r2 = rospy.Rate(40)
        self.r3 = rospy.Rate(100)
        rospy.loginfo("Iniciado nodo LOLA_interface")
        #Publications
        self.data_pub = rospy.Publisher("wheel_state", JointState, queue_size=10)
        #Subscriber
        #Check the status of the robot
        #TODO: Chequear que mensaje inicial es correcto. Si no sacar error
        self.r2.sleep()
        r = rospy.Rate(self.read_encoder_freq)
        
        #Subscriber
        rospy.Subscriber("cmd_wheel", JointState, self.callback_vel, queue_size=1)
        while not rospy.is_shutdown():
            self.read_encoders()
            r.sleep()


# Function: check
#
# Comments
# -------------------
# Check if there has been no velocity cmd_wheel message in a defined time
# if so, stop the robot
# Parameters
# ----------
#
# Returns
# -------
    def check(self):
        if (rospy.get_time() - self.lastTwistTime) > self.twistTimeout:
            #TODO: Enviar comando de parada a la silla
            self.r3.sleep() 
            rospy.loginfo("Parada por no recibir datos")

# Function: callback_vel
#
# Comments
# ----------
# Callback of the cmd_wheel topic.
# Convert the velocity in rad/s to data.
# [0-127] for positive velocity.
# [255-129] for negative velocity (129 maximun velocity).
# 0 and 128 stop the wheel.
#
# Parameters
# ----------
# msg: Data of the topic
#
# Returns
# -------
    def callback_vel(self,msg): #Recepcion de velocidades
        sign = lambda x: int(copysign(1,x))
        #Save variables
        comandD = msg.velocity[self.right] #rad/s
        #print("CmdD: "+str(comandD))
        comandI = msg.velocity[self.left] #rad/s
        #print("CmdI: "+str(comandI))
        #Convert from rad/s to data 0-127
        datod = int(round(comandD * self.kdato))
        datoi = int(round(comandI * self.kdato))
        #Saturation of data
        if abs(datod) > 127:
            datod = 127 * sign(datod)
        if abs(datoi) >127:
            datoi = 127 * sign(datoi)
        #Minimum velocity asigned
        if (abs(datod)<10 and datod <> 0):
            datod = 10 * sign(datod)
        if (abs(datoi)<10 and datoi <> 0):
            datoi = 10 * sign (datoi)
        #Negative velocity convert
        if datod<0:
            datod=256 + datod
        if datoi<0:
            datoi=256 + datoi
        #TODO: Analizar si se debe enviar algun mecanismo de eco
	self.encoderd += int(0.5+(datod/self.kdato)*(self.pasos_vuelta/(2*pi))/self.read_encoder_freq)
   	self.timerd += (1/float(self.read_encoder_freq))
        self.encoderi += int(0.5+(datoi/self.kdato)*(self.pasos_vuelta/(2*pi))/self.read_encoder_freq)
	self.timeri += (1/float(self.read_encoder_freq))
	self.r3.sleep()
	#Se toma para ver si hay un Timeout.   
        self.lastTwistTime = rospy.get_time() 
        rospy.loginfo('V'+format(int(datod),'03d')+format(datoi,'03d'))

       


# Function: read_enc
#
# Comments
# ----------
# Read the encoders of the robot
# Calculate the possition and veocity of the wheels with the encoders data
# and publish in a JointState message
#
# Parameters
# ----------
#
# Returns
# -------
    def read_encoders(self):
        rospy.loginfo("funcion lectura")

        #data = self.arduino.readline()
        # TODO: Se puede chequear que N y P son el comienzo y
        #final de la trama
        self.r2.sleep()
        data = JointState()
        #Increment calc
        dt_right = float(self.timerd - self.time_enc_right_last)
        dsteps_right = float(self.encoderd - self.steps_enc_right_last)
        self.time_enc_right_last = self.timerd
        self.steps_enc_right_last = self.encoderd
        #Posicion
        posD = (self.encoderd/self.pasos_vuelta) * 2 * pi
        #Velocidad angular
        if(dt_right <> 0):
            wd = ((dsteps_right/self.pasos_vuelta) * 2 * pi) / dt_right
        else:
            wd = 0
        dt_left = float(self.timeri-self.time_enc_left_last)
        dsteps_left = float(self.encoderi-self.steps_enc_left_last)
        #Guardiar variables actuales
        self.time_enc_left_last = self.timeri
        self.steps_enc_left_last = self.encoderi
        #Save data to publish
        data.name = ["RIGHT"]
        data.position = [posD]
        data.velocity = [wd]
        data.effort = [0]
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = "base_link"
        self.data_pub.publish(data)
        rospy.loginfo("ENCD: "+ str(dsteps_right) + " TIEMPD: "+str(dt_right*1E6))
        # Si la lectura del arduino de encoder y velocidad
        # Es en dos fases, leer aqui la info de izda
        #Calcular incrementos
        #Distancia
        posI = (self.encoderi/self.pasos_vuelta) * 2 * pi
        #Velocidad angular
        if(dt_left <> 0):
            wi = ((dsteps_left/self.pasos_vuelta) * 2 * pi) / dt_left
        else:
            wi = 0
        #Save data to publish
        data = JointState()
        data.name = ["LEFT"]
        data.position = [posI]
        data.velocity = [wi]
        data.effort = [0]
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = "base_link"
        #Publish topic
        self.data_pub.publish(data)
        rospy.loginfo("ENCI: "+ str(dsteps_left) + " TIEMPI: "+str(dt_left*1E6))


if __name__ == '__main__':
	foo = HwClass()
	foo.main()

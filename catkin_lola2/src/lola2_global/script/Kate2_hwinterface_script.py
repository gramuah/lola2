#!/usr/bin/env python

#Creador: FJR 2018
#Funcion: Interface entre el controlador hw_interface y el bus can
#Recibe: Controlador [Joint_State (commandD, commandI)],Can [Datos Encoder]
#Envia: Controlador[PosD, PosI, Wi, Wd], Can [DatoI, DatoD]
import math
import struct
from math import sin, cos, pi

import rospy

import tf

##Mensajes
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from kate_global.msg import enc_msg
from kate_global.msg import CAN
from std_msgs.msg import Header
from std_msgs.msg import Int16

class HW_CANUSB_CLASS:
# Function: __init__
#
# Comments
# ----------
# Constructor of the class HW_CANUSB_CLASS
# Define system constants.
#
# Parameters
# ----------
#
# Returns
# -------
	def __init__(self):

		#constant parameters
		#self.pasos_vuelta= 152.7
		self.pasos_vuelta= 1425.2
		self.left= 0
		self.right= 1

		self.kdato=38.13
		self.twistTimeout = 2

		self.loop_mode= 'C' #'A' oppen loop, 'C' pid control
		self.check_period = 5.0


		#Encoders data
		self.time_enc_left_last=0
		self.time_enc_right_last=0

		self.steps_enc_left_last=0
		self.steps_enc_right_last=0


		#Status variables
		self.modoPC=0

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
		rospy.loginfo("Iniciado nodo KATE_interface")

		#Publications
		self.data_pub = rospy.Publisher("wheel_state", JointState, queue_size=100)
		self.canpub = rospy.Publisher('cantx', CAN, queue_size=100)


		#Subscriber
		rospy.Subscriber("/enc", enc_msg, self.callback_enc) #One topic for both encoders
		rospy.Subscriber("cmd_wheel", JointState, self.callback_vel)#Cambiar al mensaje recibido por el control
		rospy.Subscriber("canrx", CAN, self.callback_CAN)

		rospy.loginfo("Esperando sincronizacion")

		#Check the status of the chair
		r = rospy.Rate(self.check_period)
		self.lastTwistTime = rospy.get_time()
		while not rospy.is_shutdown():
			self.check(self.modoPC)
			r.sleep()

# Function: check
#
# Comments
# ----------
# Check the status of the chair
#
# Parameters
# ----------
# Mode of the Weelchair
# 	0 Mode not selected
# 	3 Mode PC, allows to control the Weelchair
# 	Others modes. No action
# Returns
# -------
	def check(self,modo):

			if(modo==3): #Syncronization with the Weelchair.

				if (rospy.get_time() - self.lastTwistTime) > self.twistTimeout: #No command reception, stop the Weelchair

					dato = CAN()
					dato.stdId = 288
					dato.extId = -1
					dato.data = struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + 'A' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)

					self.canpub.publish(dato)

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

		#Save variables
		comandD=msg.velocity[self.right] #rad/s
		#print("CmdD: "+str(comandD))
		comandI=msg.velocity[self.left] #rad/s
		#print("CmdI: "+str(comandI))

		#Convert from rad/s to data

		datod=int(comandD*self.kdato)
		datoi=int(comandI*self.kdato)

		#Saturation of data
		if datod>127:
			datod=127
		if datod<-127:
			datod=-127

		if datoi>127:
			datoi=127
		if datoi<-127:
			datoi=-127

		#Negative velocity convert
		if datod<0:
			datod=256 + datod
		if datoi<0:
			datoi=256 + datoi

		if(self.modoPC==3): #Syncronization with the Weelchair.
			dato = CAN()
			dato.stdId = 288
			dato.extId = -1
			dato.data = struct.pack('B', datod) + struct.pack('B', datoi) + struct.pack('B', 0) + struct.pack('B', 0) + self.loop_mode + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)
			#print("DatoD: "+str(datod))
			#print("DatoI: "+str(datoi))
			self.canpub.publish(dato)

		self.lastTwistTime = rospy.get_time() #Update last velocity command time

# Function: callback_CAN
#
# Comments
# ----------
# Callback of the canrx topic.
# Read the data of syncronization when the mode pc is activated in the
# numeric keyboard.
#
# Parameters
# ----------
# msg: Data of the topic
#
# Returns
# -------
	def callback_CAN(self,msg): #Recepcion de sincronizacion del bus can
		if msg.stdId == 273:
			(self.modoPC,)= struct.unpack('B', msg.data[:1])
			self.synchronize(self.modoPC)

# Function: synchronize
#
# Comments
# ----------
# When the mode PC is selected in the numeric keyboad, send a confirmation
# message to the hardware.
#
# Parameters
# ----------
# Mode of the Weelchair
# 	0 Mode not selected
# 	3 Mode PC, allows to control the Weelchair
# 	Others modes. No action
#
# Returns
# -------
	def synchronize(self, sinc):
		if sinc == 3:
			msg = CAN()
			msg.stdId = 288
			msg.extId = -1
			msg.data = struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0) + 'C' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)
			rospy.loginfo("Sincronizado")



# Function: callback_enc
#
# Comments
# ----------
# Callback of the enc topic
# Calculate the possition and veocity of the wheels with the encoders data
# and publish in a JointState message
#
# Parameters
# ----------
# msg: Data of the topic
#
# Returns
# -------

	def callback_enc(self,msg):
		if(msg.encID==0): #EncoderA (RIGHT)

			time_enc_right=msg.time
			steps_enc_right=msg.data

			#Increment calc
			dt_right=(time_enc_right-self.time_enc_right_last)*(10**-4) #1 tick cada 100us
			#print("dtd: "+str(dt_right))
			dsteps_right=steps_enc_right-self.steps_enc_right_last
			#print("dencd: "+str(dsteps_right))

			#Guardiar variables actuales
			self.time_enc_right_last=time_enc_right
			self.steps_enc_right_last=steps_enc_right

			#Posicion
			posD=(steps_enc_right/self.pasos_vuelta)*2*pi

			#Velocidad angular
			wd=((dsteps_right/self.pasos_vuelta)*2*pi)/dt_right

			#print("wd: "+str(wd))

			#Save data to publish
			data=JointState()
			data.name= ["RIGHT"]
			data.position=[posD]
			data.velocity=[wd]
			data.effort=[0]
			data.header.stamp = rospy.Time.now()
			data.header.frame_id = "base_link"


		elif(msg.encID==1): #EncoderB (LEFT)

			time_enc_left=msg.time
			steps_enc_left=msg.data

			#Calcular incrementos
			dt_left=(time_enc_left-self.time_enc_left_last)*(10**-4) #100us
			dsteps_left=steps_enc_left-self.steps_enc_left_last

			#Guardiar variables actuales
			self.time_enc_left_last=time_enc_left
			self.steps_enc_left_last=steps_enc_left

			#Distancia
			posI=(steps_enc_left/self.pasos_vuelta)*2*pi
			#Velocidad angular
			wi=((dsteps_left/self.pasos_vuelta)*2*pi)/dt_left

			#Save data to publish
			data=JointState()
			data.name= ["LEFT"]
			data.position=[posI]
			data.velocity=[wi]
			data.effort=[0]
			data.header.stamp = rospy.Time.now()
			data.header.frame_id = "base_link"

		#Publish topic
		self.data_pub.publish(data)

if __name__ == '__main__':

	odom=HW_CANUSB_CLASS()
	odom.main()

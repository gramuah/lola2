#!/usr/bin/env python
#Hay que revisar los import por si hace falta alguno nuevo
import serial, time
import struct
import rospy
from math import copysign, pi
##Mensajes
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
        self.left = 0
        self.right = 1
        self.kdato = 36.28
        self.twistTimeout = 20
        self.read_encoder_freq = rospy.get_param('/diff_drive_controller/publish_rate',6)
        
        #Encoders data
        self.time_enc_left_last=0
        self.time_enc_right_last=0
        self.steps_enc_left_last=0
        self.steps_enc_right_last=0
        #TODO: Se debe permitir la apertura del puerto serie por
        #puerto y velocidad (parametros servidor)
        self.arduino = serial.Serial('/dev/arduino', 115200)
        time.sleep(0.01)


    def __del__(self):
        self.arduino.write(str('?').encode())
        self.arduino.close()

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
        rospy.loginfo("Iniciado nodo LOLA_interface")
        rospy.loginfo("velocidad lectura:  "+str(self.read_encoder_freq))
        #Publications
        self.data_pub = rospy.Publisher("wheel_state", JointState, queue_size=10)
        #Subscriber
        #Check the status of the robot
        #TODO: Chequear que mensaje inicial es correcto. Si no sacar error
        data = self.arduino.readline()
        rospy.loginfo(data)
        r = rospy.Rate(self.read_encoder_freq)
        self.lastTwistTime = rospy.get_time()
        self.dum_first_read()
        #Subscriber
        r.sleep()
        rospy.Subscriber("cmd_wheel", JointState, self.callback_vel, queue_size=1)

        while not rospy.is_shutdown():
            self.check()
            self.read_encoders()
            r.sleep()


    def dum_first_read(self):
        rospy.loginfo("LECTURA DUMMY")
        self.arduino.write(str('N').encode())
        #data = self.arduino.readline()
        # TODO: Se puede chequear que N y P son el comienzo y
        #final de 3la trama
        comienzo = self.arduino.readline()
        steps_enc_left = struct.unpack('i',self.arduino.read(4))[0]        
        time_enc_left = struct.unpack('i',self.arduino.read(4))[0]     
        steps_enc_right = struct.unpack('i',self.arduino.read(4))[0]
        time_enc_right = struct.unpack('i',self.arduino.read(4))[0]
        final = self.arduino.readline()                       
        self.time_enc_right_last = time_enc_right
        self.steps_enc_right_last = steps_enc_right
        self.time_enc_left_last = time_enc_left
        self.steps_enc_left_last = steps_enc_left
        rospy.loginfo(str(steps_enc_right))
        

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
            self.arduino.write(str('?').encode())
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
        if (abs(datod)<10 and datod != 0):
            datod = 10 * sign(datod)
        if (abs(datoi)<10 and datoi != 0):
            datoi = 10 * sign (datoi)
        #Negative velocity convert
        if datod<0:
            datod=256 + datod
        if datoi<0:
            datoi=256 + datoi
        #TODO: Analizar si se debe enviar algun mecanismo de eco
        self.arduino.write(('V'+format(int(datod),'03d')+
                           format(datoi,'03d')).encode())
        #Se toma para ver si hay un Timeout.   
        self.lastTwistTime = rospy.get_time() 
        

       


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
        self.arduino.write(str('N').encode())
        #data = self.arduino.readline()
        # TODO: Se puede chequear que N y P son el comienzo y
        #final de 3la trama
        comienzo = self.arduino.readline()
        steps_enc_left = struct.unpack('i',self.arduino.read(4))[0]        
        time_enc_left = struct.unpack('i',self.arduino.read(4))[0]     
        steps_enc_right = struct.unpack('i',self.arduino.read(4))[0]
        time_enc_right = struct.unpack('i',self.arduino.read(4))[0]
        final = self.arduino.readline()
        data = JointState()
        #Increment calc
        dt_right = float((time_enc_right - self.time_enc_right_last) * (10**-6))
        dsteps_right = float(steps_enc_right - self.steps_enc_right_last)
        self.time_enc_right_last = time_enc_right
        self.steps_enc_right_last = steps_enc_right
        #Posicion
        posD = (steps_enc_right/self.pasos_vuelta) * 2 * pi
        #Velocidad angular
        wd = ((dsteps_right/self.pasos_vuelta) * 2 * pi) / dt_right
        #Save data to publish.---9*--
        data.name = ["RIGHT"]
        data.position = [posD]
        data.velocity = [wd]
        data.effort = [0]
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = "base_link"
        self.data_pub.publish(data)
        #rospy.loginfo("Pasos encoder: "+str(steps_enc_right))
        #rospy.loginfo("ENCD: "+ str(dsteps_right) + " TIEMPD: "+str(dt_right))
        # Si la lectura del arduino de encoder y velocidad
        # Es en dos fases, leer aqui la info de izda
        #Calcular incrementos
        dt_left = float((time_enc_left-self.time_enc_left_last)*(10**-6))
        dsteps_left = float(steps_enc_left-self.steps_enc_left_last)
        #Guardiar variables actuales
        self.time_enc_left_last = time_enc_left
        self.steps_enc_left_last = steps_enc_left
        #Distancia
        posI = (steps_enc_left/self.pasos_vuelta) * 2 * pi
        #Velocidad angular
        wi = ((dsteps_left/self.pasos_vuelta) * 2 * pi) / dt_left
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
        rospy.loginfo("WI: %.2f   WD: %.2f",wi,wd)
        #rospy.loginfo("Pasos encoder: "+str(steps_enc_left))
        #rospy.loginfo("ENCI: "+ str(dsteps_left) + " TIEMPI: "+str(dt_left))


if __name__ == '__main__':
	foo = HwClass()
	foo.main()

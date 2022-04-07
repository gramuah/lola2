#!/usr/bin/env python

import math
import rospy
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class SquareMove(object):
    """
    This class is an abstract class to control a square trajectory on the turtleBot.
    It mainly declare and subscribe to ROS topics in an elegant way.
    """

    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "square_move"
        self.odom_sub_name = "/odom"
        self.vel_pub_name = "/cmd_vel"
        self.vel_pub = None
        self.odometry_sub = None
        self.start = False

        # ROS params
        self.publish_freq =  rospy.get_param('/diff_drive_controller/publish_rate',10)
        self.queue_size = 1
        self.linear_velocity = rospy.get_param('/square/linear_velocity',0.2)
        self.angular_velocity = rospy.get_param('/square/angular_velociy',1.0)
        self.square_size = rospy.get_param('/square/square_size',2.5)

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None

    def start_ros(self):

        # Create a ROS node with a name for our program
        rospy.init_node(self.node_name, log_level=rospy.INFO)

        # Define a callback to stop the robot when we interrupt the program (CTRL-C)
        rospy.on_shutdown(self.stop_robot)

        # Create the Subscribers and Publishers
        self.odometry_sub = rospy.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.joystick_sub = rospy.Subscriber("/joy", Joy, callback=self.__joys_ros_sub, queue_size=self.queue_size)
        self.vel_pub = rospy.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

    def stop_robot(self):

        r = rospy.Rate(self.publish_freq)
        # We publish for a second to be sure the robot receive the message
        while not rospy.is_shutdown():
            
            self.vel_ros_pub(Twist())
            r.sleep()

        

    def move(self):
        """ To be surcharged in the inheriting class"""
        r = rospy.Rate(self.publish_freq)

        while not rospy.is_shutdown():
            r.sleep()
    def __joys_ros_sub(self,msg):
        if(msg.buttons[0]==1):
            self.start = True
    def __odom_ros_sub(self, msg):

        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):

        self.vel_pub.publish(msg)


class SquareMoveVel(SquareMove):
   
    def __init__(self):
        
        super(SquareMoveVel, self).__init__()

    def go_forward(self, duration, speed):

        # Get the initial time
        self.t_init = time.time()
        r = rospy.Rate(self.publish_freq)

        # Set the velocity forward and wait (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            r.sleep()

    def turn(self, duration, ang_speed):

         # Get the initial time
        self.t_init = time.time()
        r = rospy.Rate(self.publish_freq)
        # Set the velocity forward and wait 2 sec (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not rospy.is_shutdown():

            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = ang_speed
            self.vel_ros_pub(msg)
            r.sleep()

    def move(self):
        straigth_duration = self.linear_velocity / self.square_size
        angle_duration = self.angular_velocity / (math.pi/2)
        self.go_forward(straigth_duration, self.linear_velocity)
        self.turn(angle_duration, self.angular_velocity)
        self.go_forward(straigth_duration, self.linear_velocity)
        self.turn(angle_duration, self.angular_velocity)
        self.go_forward(straigth_duration, self.linear_velocity)
        self.turn(angle_duration, self.angular_velocity)
        self.go_forward(straigth_duration, self.linear_velocity)
        self.turn(angle_duration, self.angular_velocity)
        self.stop_robot()


class SquareMoveOdom(SquareMove):
    """
    This class implements a semi closed-loop square trajectory based on relative position control,
    where only odometry is used. HOWTO:
     - Start the roscore (on the computer or the robot, depending on your configuration)
            $ roscore
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
     - Start this node on your computer:
            $ python move_square odom
    """

    def __init__(self):


        super(SquareMoveOdom, self).__init__()

        

    def get_z_rotation(self, orientation):

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        if(yaw <= 0):
            yaw += 2*math.pi
        if(yaw >= 2*math.pi):
            yaw = yaw - 2*math.pi
        
        return yaw
        
    def move_of(self, d, speed=0.5):

        epsilon = 0.005
        kp_lin_speed_limit = 0.2
        kp_lin_speed = 1
        x_init = self.odom_pose.position.x
        y_init = self.odom_pose.position.y
        r = rospy.Rate(self.publish_freq)
        # Set the velocity forward until distance is reached
        while (not rospy.is_shutdown()):
            diferencia = (d - math.sqrt((self.odom_pose.position.x - x_init)**2 + 
                         (self.odom_pose.position.y - y_init)**2))
            if (diferencia < epsilon):
                break
#            if (diferencia < kp_lin_speed_limit):
#-                kp_lin_speed = diferencia / kp_lin_speed_limit
            msg = Twist()
            msg.linear.x = speed * kp_lin_speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            r.sleep()

        

    def turn_of(self, a, ang_speed=0.4):

        # Convert the orientation quaternion message to Euler angles
        epsilon = 0.004
        kp_ang_speed_limit = 10 *math.pi/180.0
        kp_ang_speed = 1
        a_init = self.get_z_rotation(self.odom_pose.orientation)
        r = rospy.Rate(self.publish_freq)
        a_fin = a_init + a
        if (a_fin < 0):
            a_fin += 2*math.pi
        if (a_fin >= 2*math.pi):
            a_fin -= 2*math.pi
        rospy.loginfo("Inicial: %.3f, Final: %.3f", a_init, a_fin)
        # Set the angular velocity forward until angle is reached
        if (a>0 and a_fin>a_init): #Giro a izquierdas
            while (not rospy.is_shutdown()):
                diferencia = (a_fin - 
                              self.get_z_rotation(self.odom_pose.orientation))
                rospy.loginfo("Actual: %.3f  Diferencia: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)
                if (diferencia < epsilon):
                    break
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                msg.angular.z = ang_speed * kp_ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()
        elif(a>0):
            while (not rospy.is_shutdown()):
                diferencia = (2*math.pi -
                              self.get_z_rotation(self.odom_pose.orientation)) 
                rospy.loginfo("Actual: %.3f  Diferencia a Cero: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)                             
                if (diferencia < 0 or
                    self.get_z_rotation(self.odom_pose.orientation) <
                    (a_init-epsilon)):
                    break
                if (diferencia + a_fin < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                msg.angular.z = ang_speed * kp_ang_speed
                msg = Twist()
                msg.angular.z = ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()
            while (not rospy.is_shutdown()):
                diferencia = (a_fin - 
                              self.get_z_rotation(self.odom_pose.orientation)) 
                rospy.loginfo("Actual: %.3f  Diferencia: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)
                if (diferencia < epsilon):
                    break
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                msg.angular.z = ang_speed * kp_ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()
        elif(a<0 and a_fin < a_init):
            diferencia_init = a_init - a_fin
            while (not rospy.is_shutdown()):
                diferencia = (self.get_z_rotation(self.odom_pose.orientation) -
                              a_fin)
                rospy.loginfo("Actual: %.3f  Diferencia: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)
                if(diferencia < epsilon or diferencia>(diferencia_init+epsilon)):
                    break
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                msg.angular.z = -ang_speed * kp_ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()

        elif(a<0):
            while (not rospy.is_shutdown()):
                if (self.get_z_rotation(self.odom_pose.orientation) > 
                    (a_init+epsilon)):
                    break;
                diferencia = (self.get_z_rotation(self.odom_pose.orientation) + 
                    (2*math.pi - a_fin)) 
                rospy.loginfo("Actual: %.3f  Diferencia a Cero: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)                             
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit    
                msg = Twist()
                msg.angular.z = -ang_speed*kp_ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()
            diferencia_init = (self.get_z_rotation(self.odom_pose.orientation) -
                               a_fin)
            while (not rospy.is_shutdown()):
                diferencia = (self.get_z_rotation(self.odom_pose.orientation) -
                              a_fin)
                rospy.loginfo("Actual: %.3f  Diferencia: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)
                if(diferencia < epsilon or diferencia>(diferencia_init+epsilon)):
                    break
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                msg.angular.z = -ang_speed * kp_ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()

    def calculate_angles(self, giro = 1):
        angles = [None]*4
        for i in range(4):
            angles[i] = (self.get_z_rotation(self.odom_pose.orientation) +
                          giro*math.pi*(i+1)/2)
            
        return angles
    
    def calculate_turn_angle (self,final_angle,giro = 1):
        actual_angle = self.get_z_rotation(self.odom_pose.orientation)
        if (giro == 1):
            if (actual_angle >final_angle):
                actual_angle -= 2*math.pi
        angle_to_turn = final_angle - actual_angle
        if angle_to_turn > 2*math.pi:
            angle_to_turn -= 2*math.pi
        return angle_to_turn
    

    def move(self):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not rospy.is_shutdown():
            time.sleep(0.1)

        # giro = 1 izquierdas, giro = -1 derechas
        angles = self.calculate_angles(1)           
        self.move_of(self.square_size,self.linear_velocity)
#        angle = self.calculate_turn_angle(angles[0])
#        self.turn_of(angle,self.angular_velocity)
#        self.move_of(self.square_size,self.linear_velocity)
#       angle = self.calculate_turn_angle(angles[1])
#        self.turn_of(angle,self.angular_velocity)
#        self.move_of(self.square_size,self.linear_velocity)
#        angle = self.calculate_turn_angle(angles[2])
#        self.turn_of(angle,self.angular_velocity)
#        self.move_of(self.square_size,self.linear_velocity)
#        angle = self.calculate_turn_angle(angles[3])
#        self.turn_of(angle,self.angular_velocity)
        self.stop_robot()


if __name__ == '__main__':

    # Choose the example you need to run in the command line
    type = rospy.get_param('/square/type',"odom")

    if type == "vel":
        r = SquareMoveVel()
    elif type == "odom":
        r = SquareMoveOdom()
    else:
        sys.exit(-1)
            # Listen and Publish to ROS + execute moving instruction
    r.start_ros()
    rospy.loginfo("Nodo ROS inicializado")
    while(r.start == False):
        pass
    rospy.loginfo("Boton recibido")
    r.move()

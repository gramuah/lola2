#!/usr/bin/env python

import math
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



class MoveDiscrete(object):
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
        self.linear_velocity = rospy.get_param('/discrete_move/linear_velocity',0.2)
        self.angular_velocity = rospy.get_param('/discrete_move/angular_velociy',0.5)
        self.straight_distance = rospy.get_param('/discrete_move/straight_distance',1)
        self.deep_angle_division = rospy.get_param('/discrete_move/deep_angle_division',2)
        r = 2 ** self.deep_angle_division 
        self.possible_angles = ([float(i) * math.pi* (float(2)/float(r))
                                for i in range(0,r)])
        self.angle_pointer = 0

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None

    def start_ros(self):

        # Create a ROS node with a name for our program
        rospy.init_node(self.node_name, log_level=rospy.INFO)

        # Define a callback to stop the robot when we interrupt the program (CTRL-C)
        rospy.on_shutdown(self.stop_robot)

        # Create the Subscribers and Publishers
        self.odometry_sub = rospy.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.vel_pub = rospy.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

    def stop_robot(self):

        r = rospy.Rate(self.publish_freq)
        # We publish for a second to be sure the robot receive the message
        while not rospy.is_shutdown():
            
            self.vel_ros_pub(Twist())
            r.sleep()

       

    
    
    def __odom_ros_sub(self, msg):
        if(self.odom_pose == None):
            (r_, p_, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                   msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z,
                                                   msg.pose.pose.orientation.w])
            a = [yaw + i for i in self.possible_angles]
            self.possible_angles = ([i - 2*math.pi if i>= 2*math.pi else i
                                     for i in a])
        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):

        self.vel_pub.publish(msg)



class MoveOdom(MoveDiscrete):
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


        super(MoveOdom, self).__init__()

        

    def get_z_rotation(self, orientation):

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        if(yaw <= 0):
            yaw += 2*math.pi
        if(yaw >= 2*math.pi):
            yaw = yaw - 2*math.pi
        
        return yaw
        
    def move_straight(self,steps = 1,fordward = 1):
        d = self.straight_distance * steps
        speed = self.linear_velocity
        epsilon = 0.005
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
            msg.linear.x = fordward * speed * kp_lin_speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            r.sleep()

        

    def turn_of(self, a):

        # Convert the orientation quaternion message to Euler angles
        ang_speed = self.angular_velocity
        epsilon = 0.08
        epsilon2 = 0.1
        kp_ang_speed_limit = 10 *math.pi/180.0
        kp_ang_speed = 1
        a_init = self.get_z_rotation(self.odom_pose.orientation)
        r = rospy.Rate(self.publish_freq)
        a_fin = a_init + a
        if (a_fin < 0):
            a_fin += 2*math.pi
        if (a_fin >= 2*math.pi):
            a_fin -= 2*math.pi
        """rospy.loginfo("Inicial: %.3f, Final: %.3f", a_init, a_fin)"""
        diferencia_old = 1E6;
        # Set the angular velocity forward until angle is reached
        if (a>0 and a_fin>a_init): #Giro a izquierdas
            while (not rospy.is_shutdown()):
                diferencia = (a_fin - 
                              self.get_z_rotation(self.odom_pose.orientation))
                """ rospy.loginfo("Actual: %.3f  Diferencia: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)"""
                if (diferencia < epsilon or 
                    diferencia > (diferencia_old+epsilon2)):
                    break
                diferencia_old = diferencia
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                #msg.angular.z = ang_speed * kp_ang_speed
                msg.angular.z = ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()
        elif(a>0):
            while (not rospy.is_shutdown()):
                diferencia = (2*math.pi -
                              self.get_z_rotation(self.odom_pose.orientation)) 
                """rospy.loginfo("Actual: %.3f  Diferencia a Cero: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)"""
                if (diferencia < 0 or
                    diferencia > (diferencia_old+epsilon2)):
                    break
                diferencia_old = diferencia
                if (diferencia + a_fin < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                msg.angular.z = ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()
            while (not rospy.is_shutdown()):
                diferencia = (a_fin - 
                              self.get_z_rotation(self.odom_pose.orientation)) 
                """rospy.loginfo("Actual: %.3f  Diferencia: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)"""
                if (diferencia < epsilon or 
                    diferencia > (diferencia_old+epsilon2)):
                    break
                diferencia_old = diferencia
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                #msg.angular.z = ang_speed * kp_ang_speed
                msg.angular.z = ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()
        elif(a<0 and a_fin < a_init):
            diferencia_init = a_init - a_fin
            while (not rospy.is_shutdown()):
                diferencia = (self.get_z_rotation(self.odom_pose.orientation) -
                              a_fin)
                """rospy.loginfo("Actual: %.3f  Diferencia: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)"""
                if(diferencia < epsilon or diferencia>(diferencia_init+epsilon2)):
                    break
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                #msg.angular.z = -ang_speed * kp_ang_speed
                msg.angular.z = -ang_speed 
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
                """rospy.loginfo("Actual: %.3f  Diferencia a Cero: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)  """                           
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit    
                msg = Twist()
                #msg.angular.z = -ang_speed*kp_ang_speed
                msg.angular.z = -ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()
            diferencia_init = (self.get_z_rotation(self.odom_pose.orientation) -
                               a_fin)
            while (not rospy.is_shutdown()):
                diferencia = (self.get_z_rotation(self.odom_pose.orientation) -
                              a_fin)
                """rospy.loginfo("Actual: %.3f  Diferencia: %.3f",self.get_z_rotation(self.odom_pose.orientation),diferencia)"""
                if(diferencia < epsilon or diferencia>(diferencia_init+epsilon)):
                    break
                if (diferencia < kp_ang_speed_limit):
                    kp_ang_speed = diferencia / kp_ang_speed_limit
                msg = Twist()
                #msg.angular.z = -ang_speed * kp_ang_speed
                msg.angular.z = -ang_speed
                msg.linear.x = 0
                self.vel_ros_pub(msg)
                r.sleep()

      
    def calculate_turn_angle (self,final_angle,giro = 1):
        actual_angle = self.get_z_rotation(self.odom_pose.orientation)
        if (giro == 1):
            if (actual_angle >final_angle):
                actual_angle -= 2*math.pi
        if (giro == -1):
            if(final_angle > actual_angle):
                actual_angle += 2*math.pi
        angle_to_turn = final_angle - actual_angle
        if angle_to_turn > 2*math.pi:
            angle_to_turn -= 2*math.pi
        if angle_to_turn < -2* math.pi:
            angle_to_turn += 2*math.pi
        return angle_to_turn
    
    def turn_right_a(self):
        self.turn_of(-math.pi/2)
        
    def turn_right_b(self,steps = 1):
        angle_pointer = (self.angle_pointer - steps) % len(self.possible_angles)
        final_angle = self.possible_angles[angle_pointer]
        self.angle_pointer = self.possible_angles.index(final_angle)
        angle_to_turn = self.calculate_turn_angle(final_angle,-1)
        """rospy.loginfo("Angulo Final: %.3f  Angulo a Girar: %.3f",
                      final_angle,angle_to_turn)"""
        self.turn_of(angle_to_turn)
    
    def turn_left_b(self,steps = 1):
        self.angle_pointer = (self.angle_pointer + steps) % len(self.possible_angles)
        final_angle = self.possible_angles[self.angle_pointer]
        angle_to_turn = self.calculate_turn_angle(final_angle,1)
        """rospy.loginfo("Angulo Final: %.3f  Angulo a Girar: %.3f",
                      final_angle,angle_to_turn)"""
        self.turn_of(angle_to_turn)
        
    def turn_left_a(self):
        self.turn_of(math.pi/2)
        
    def move_backward(self,steps = 1):
        self.move_straight(steps,-1)
    
    def stop(self):
        r = rospy.Rate(self.publish_freq)
        msg = Twist()
        msg.angular.z = 0
        msg.linear.x = 0
        self.vel_ros_pub(msg)
        r.sleep()
        
    



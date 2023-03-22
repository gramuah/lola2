#!/usr/bin/env python3
import sys
import cv2
import time

import rospy
from discrete_move.srv import DiscreteServer, DiscreteServerResponse
from odometry import Odom

from habitat_baselines.class_emb import embclip
from image_preprocessing import ImagePreprocessing


def send_action_service(mov: str, ang: int, service_name: str = "discrete_move") -> bool:
    rospy.wait_for_service(service_name, timeout=10.0)

    try:
        serv = rospy.ServiceProxy(service_name, DiscreteServer)
        resp = serv(mov, ang)
        return resp.response

    except (rospy.ServiceException, rospy.ROSException):
        return False


if __name__ == "__main__":
    rospy.init_node('vsn_node', anonymous=True)

    action_dict = {0: "Stop",
                   1: "Forward",
                   2: "Left",
                   3: "Right",
                   4: "Backward",
                   5: "Forward", }

    # Load configure svn
    object_goal = rospy.get_param('/vsn/goal')
    sub_name = rospy.get_param('/vsn/camera_topic')

    preprocess = ImagePreprocessing(sub_name)
    model = embclip()
    odometry = Odom()

    rospy.loginfo(f"I'm searching a Chair \n")

    while 1:
        image = preprocess.get_image_rgb()
        if image is not None:

            # IA class
            position = odometry.get_actual_position()
            action = model.train(image, object_goal, position)

            action = action_dict[action[0]]
            rospy.loginfo(f"Movement: {action}")

            server_response = send_action_service(action, 30)
            time.sleep(0.5)

            if not server_response:
                rospy.loginfo("An error has been detected from the server")
                break

            if action == "Stop":
                rospy.loginfo("Robot reached the goal satisfactorily")
                sys.exit()

    sys.exit()

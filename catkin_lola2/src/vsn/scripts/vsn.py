#!/usr/bin/env python3

import sys
import rospy
from discrete_move.srv import DiscreteServer, DiscreteServerResponse

from image_preprocessing import ImagePreprocessing
from ia import Ia


def send_action_service(mov: str, ang: int) -> bool:
    rospy.wait_for_service("discrete_move", 5.0)
    print("Hasta aquí llego")

    try:
        serv = rospy.ServiceProxy("discrete_move", DiscreteServer)
        resp1 = serv(mov, ang)
        print("¿Hola?")
        return resp1.response

    except (rospy.ServiceException, rospy.ROSException):
        return False


if __name__ == "__main__":

    # Initialize svn_node
    """
    if len(sys.argv) != 2:

        print()
        print("\x1b[31m"+"ERROR")
        print("You should write rosrun service_node svn.py and the goal of the AI")
        print("\x1b[37m")
        sys.exit()
    """
    #else:
    print("I'm searching a " + "chair")
    #print("I'm searching a " + rospy.get_param('/vsn/goal'))
    preprocess = ImagePreprocessing(sub_name="/camera/color/image_raw")
    rospy.init_node('vsn_node', anonymous=True)
    ia = Ia()

    while 1:
        image = preprocess.get_image_rgb()
        if image is not None:
            # IA class
            action, angle = ia.random()
            if send_action_service(action, angle) is not True:
                print("Aqui he llegado")
                break

            out = input("Press Enter to continue or q to exit...")

            if out == "q":
                break

    print("Robot has stopped ")

    sys.exit()

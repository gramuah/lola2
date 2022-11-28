import rospy
from discrete_move.srv import DiscreteServer, DiscreteServerResponse

import time
from move_robot import MoveRobot


class Server:
    def __init__(self, serv_name: str = "discrete_move") -> None:
        # self.robot = MoveRobot()

        self.serv_name = serv_name
        rospy.Service(self.serv_name, DiscreteServer, self._callback_action)
        rospy.loginfo("Service up, waiting action")
        rospy.spin()

    def _callback_action(self, req) -> int:

        #while self.robot.position_robot is None and not rospy.is_shutdown():
        #    time.sleep(0.1)

        if req.movement == 'Forward':
            print("Forward")
            # self.robot.move_forward()

        elif req.movement == 'Left':
            print("Left", req.angle)
            # self.robot.turn_left(req.angle)

        elif req.movement == 'Right':
            print("Right", req.angle)
            # self.robot.turn_right(req.angle)

        elif req.movement == 'Stop':
            print("Stop")
            return DiscreteServerResponse(False)
            # self.robot.stop_robot()

        else:
            print("Problem")
            return DiscreteServerResponse(False)

        return DiscreteServerResponse(True)

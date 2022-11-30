import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImagePreprocessing:
    """
    This class is in charge of reading image topic, and convert it to a ROS message from a numpy array
    """
    def __init__(self, sub_name: str = "/camera/color/image_raw") -> None:
        self.bridge = CvBridge()
        self._sub_name: str = sub_name
        self._rgb_image = None

        rospy.loginfo("Connecting camera...")
        # ROS subscriber to the camera topic, if its another one we will need to change its value
        rospy.Subscriber(self._sub_name, Image, self._callback_rgb)

    def _callback_rgb(self, data: Image) -> np.array:
        try:
            self._rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

    def get_image_rgb(self):
        return self._rgb_image

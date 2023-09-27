"""
An action client which can be called by other nodes which processes feed from an image stream
using OpenCV

Replace the 'count_magnets' method with your own code
"""
from typing import Tuple, Union

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from mtc_panda_interfaces.action import InspectMagnets
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class InspectAction(Node):

    def __init__(self):
        super().__init__('visual_inspection')  # type: ignore

        # Subscribers and publishers
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic='camera/color/image_raw',
            callback=self.camera_cb,
            qos_profile=qos_profile_system_default)

        self.output_pub = self.create_publisher(
            msg_type=Image,
            topic='~/image_processed',
            qos_profile=qos_profile_system_default)

        # Inspect action server
        self.inspect_server = ActionServer(
            self,
            action_type=InspectMagnets,
            action_name='inspect_magnets',
            execute_callback=self.inspect_action_cb)

        # Other image processing attributes
        self.last_imgmsg: Union[Image, None] = None
        self.cv_bridge = CvBridge()

    def camera_cb(self, msg: Image):
        """
        Stores the most recent image message from the camera
        """
        self.last_imgmsg = msg

    def inspect_action_cb(self, goal_handle):
        """
        Action server callback
        """
        self.get_logger().info('Executing inspect action...')
        result = InspectMagnets.Result()
        if self.last_imgmsg is not None:
            image = self.cv_bridge.imgmsg_to_cv2(self.last_imgmsg, desired_encoding='bgr8')
            count, annotated = self.count_magnets(image)
            self.output_pub.publish(self.cv_bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
            goal_handle.succeed()
            result.magnets_found = Int32(data=count)
        else:
            pass

        return result

    def count_magnets(self, img: np.ndarray) -> Tuple[int, np.ndarray]:
        """
        Method for counting number of magnets in the ROI and returning an annotated image
        """
        count = 40
        annotation = cv2.putText(
            img, f"{self.get_clock().now().seconds_nanoseconds()[0]}",
            (40, 680), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 0), 2)
        return count, annotation


def main(args=None):
    rclpy.init(args=args)
    inspect_action = InspectAction()
    rclpy.spin(inspect_action)
    inspect_action.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

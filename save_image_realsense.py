# subscribe to realsense camera and save images

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml
import tf.transformations as tf_transformations
from autolab_core import RigidTransform
import time


# Callback function for the realsense camera
if __name__ == "__main__":
    def callback(data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Image window", cv_image)

        # save the image to a file upon key press
        if cv2.waitKey(1) & 0xFF == ord('s'):
            cv2.imwrite(f"image_{time.time()}.jpg", cv_image)
            print("Image saved!")

    # Initialize the node
    rospy.init_node('save_image_realsense', anonymous=True)

    # Subscribe to the realsense camera
    rospy.Subscriber("/camera/color/image_raw", Image, callback)

    # Keep the node running
    rospy.spin()

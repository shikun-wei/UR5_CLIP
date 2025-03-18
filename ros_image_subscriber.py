#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage

class ROSImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.latest_image = None

    def image_callback(self, msg):
        try:
            encoding = msg.encoding
            # rospy.loginfo(f"[INFO] Received image with encoding: {encoding}")

            if encoding == "mono8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.latest_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            # rospy.loginfo("[INFO] Image successfully converted.")

        except Exception as e:
            rospy.logerr(f"[ERROR] Image conversion error: {str(e)}")

    def get_latest_image(self):
        return self.latest_image

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
        
        # 订阅RGB图像
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        # 订阅深度图
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        self.latest_rgb = None
        self.latest_depth = None

    def image_callback(self, msg):
        """ 处理RGB图像 """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_rgb = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        except Exception as e:
            rospy.logerr(f"[ERROR] RGB Image conversion error: {str(e)}")

    def depth_callback(self, msg):
        """ 处理深度图 """
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")  # 深度图通常是 16 位
            self.latest_depth = cv_depth
        except Exception as e:
            rospy.logerr(f"[ERROR] Depth Image conversion error: {str(e)}")

    def get_latest_images(self):
        """ 返回 RGB 图像和深度图 """
        return self.latest_rgb, self.latest_depth
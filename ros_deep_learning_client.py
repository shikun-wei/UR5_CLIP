#!/usr/bin/env python3
import rospy
import socket
import struct
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scripts.ros_image_subscriber import ROSImageSubscriber  

class ROSDeepLearningClient:
    def __init__(self, server_ip="172.17.0.1", port=9999):
        rospy.init_node("deep_learning_client", anonymous=True)
        self.image_subscriber = ROSImageSubscriber()
        self.bridge = CvBridge()
        self.server_ip = server_ip
        self.port = port

    def send_images_to_server(self, rgb_image, depth_image):
        """ 发送 RGB 图像和深度图到服务器 """
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            client.connect((self.server_ip, self.port))

            # 转换 RGB 图像
            rgb_np = np.array(rgb_image)
            rgb_np = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)

            # 压缩 RGB 图像
            success_rgb, encoded_rgb = cv2.imencode('.jpg', rgb_np, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not success_rgb:
                rospy.logerr("[ERROR] RGB Image encoding failed!")
                return
            
            # 压缩深度图（使用 PNG 保留信息）
            success_depth, encoded_depth = cv2.imencode('.png', depth_image)
            if not success_depth:
                rospy.logerr("[ERROR] Depth Image encoding failed!")
                return

            # 发送 RGB 数据大小
            client.sendall(struct.pack(">L", len(encoded_rgb.tobytes())))
            client.sendall(encoded_rgb.tobytes())

            # 发送 深度图 数据大小
            client.sendall(struct.pack(">L", len(encoded_depth.tobytes())))
            client.sendall(encoded_depth.tobytes())

            rospy.loginfo("[INFO] Images sent successfully.")

            # ✅ 确保正确接收 "DONE"
            while True:
                done_signal = client.recv(4)
                if not done_signal:
                    rospy.logwarn("[WARNING] Connection closed unexpectedly while waiting for DONE")
                    break
                if done_signal == b"DONE":
                    rospy.loginfo("[INFO] Server finished processing. Ready for next image.")
                    break
                rospy.logwarn(f"[WARNING] Unexpected response from server: {done_signal}")

        except Exception as e:
            rospy.logerr(f"[ERROR] Communication error: {e}")

        finally:
            client.close()
            rospy.loginfo("[INFO] Connection closed.")

    def run(self):
        """ 逐步发送图像 """
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            latest_rgb, latest_depth = self.image_subscriber.get_latest_images()
            if latest_rgb is not None and latest_depth is not None:
                rospy.loginfo("[INFO] Sending RGB & Depth image to server.")
                self.send_images_to_server(latest_rgb, latest_depth)
            else:
                rospy.logwarn("[WARNING] No images received yet.")
            rate.sleep()

if __name__ == "__main__":
    rospy.loginfo("[INFO] ROS Deep Learning Client started.")
    client = ROSDeepLearningClient()
    client.run()

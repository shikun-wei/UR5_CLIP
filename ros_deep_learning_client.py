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
import threading

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scripts.ros_image_subscriber import ROSImageSubscriber  # Importing image subscriber

class ROSDeepLearningClient:
    def __init__(self, server_ip="172.17.0.1", port=9999):
        rospy.init_node("deep_learning_client", anonymous=True)
        self.image_subscriber = ROSImageSubscriber()
        self.bridge = CvBridge()
        self.server_ip = server_ip
        self.port = port

    def send_image_to_server(self, image_pil):
        """ 发送图像到服务器，并等待服务器处理完成 """
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            client.connect((self.server_ip, self.port))

            # Convert PIL Image to NumPy Array
            image_np = np.array(image_pil)
            image_np = cv2.cvtColor(np.array(image_pil), cv2.COLOR_RGB2BGR)

            # 压缩图像为 JPEG
            success, encoded_image = cv2.imencode('.jpg', image_np, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not success:
                rospy.logerr("[ERROR] Image encoding failed!")
                return
            
            data = encoded_image.tobytes()
            data_size = struct.pack(">L", len(data))  # 发送 4 字节表示数据大小

            rospy.loginfo(f"[INFO] Sending image of size: {len(data)} bytes")
            client.sendall(data_size + data)  # 发送数据大小和图像数据

            # ✅ 确保正确接收 "DONE"
            while True:
                done_signal = client.recv(4)
                if not done_signal:
                    rospy.logwarn("[WARNING] Connection closed unexpectedly while waiting for DONE")
                    break
                if done_signal == b"DONE":
                    rospy.loginfo("[INFO] Server finished processing. Ready for next image.")
                    break  # ✅ 收到 DONE 后退出循环
                rospy.logwarn(f"[WARNING] Unexpected response from server: {done_signal}")

        except Exception as e:
            rospy.logerr(f"[ERROR] Communication error: {e}")

        finally:
            client.close()
            rospy.loginfo("[INFO] Connection closed.")

    def run(self):
        """ 逐步发送图像，等待服务器处理完成后再发送下一张 """
        rate = rospy.Rate(0.5)  # 订阅频率，防止过快发送
        while not rospy.is_shutdown():
            latest_image = self.image_subscriber.get_latest_image()
            if latest_image is not None:
                rospy.loginfo("[INFO] Sending latest image to server.")
                self.send_image_to_server(latest_image)
            else:
                rospy.logwarn("[WARNING] No image received yet.")
            rate.sleep()  # 控制发送间隔


if __name__ == "__main__":
    rospy.init_node("deep_learning_client", anonymous=True)  # Initialize ROS node
    rospy.loginfo("[INFO] ROS Deep Learning Client started.")
    client = ROSDeepLearningClient()
    client.run()

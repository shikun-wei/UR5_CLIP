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
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32


sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from scripts.ros_image_subscriber import ROSImageSubscriber  

class ROSDeepLearningClient:
    def __init__(self, server_ip="172.17.0.1", port=9999):
        rospy.init_node("deep_learning_client", anonymous=True)
        self.image_subscriber = ROSImageSubscriber()
        self.bridge = CvBridge()
        self.server_ip = server_ip
        self.port = port
        self.dynamic_obstacle_pub = rospy.Publisher("/dynamic_obstacle_pos", PointCloud, queue_size=10)

    def send_images_to_server(self, rgb_image, depth_image):
        """发送 RGB 图像和深度图到服务器，并接收物体桌面坐标"""
        import json  # 确保引入
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

            # 发送 RGB 图像
            client.sendall(struct.pack(">L", len(encoded_rgb)))
            client.sendall(encoded_rgb.tobytes())

            # 发送 深度图
            client.sendall(struct.pack(">L", len(encoded_depth)))
            client.sendall(encoded_depth.tobytes())

            rospy.loginfo("[INFO] Images sent successfully. Waiting for response...")

            # ✅ 接收回传数据长度（前 4 字节）
            len_bytes = client.recv(4)
            if not len_bytes:
                rospy.logerr("[ERROR] Failed to receive response length.")
                return

            response_len = struct.unpack(">L", len_bytes)[0]

            # ✅ 接收实际 JSON 数据
            response_bytes = b""
            while len(response_bytes) < response_len:
                packet = client.recv(response_len - len(response_bytes))
                if not packet:
                    rospy.logerr("[ERROR] Connection closed before full response received.")
                    return
                response_bytes += packet

            # ✅ 解码为 Python 对象
            coords_json = response_bytes.decode('utf-8')
            desktop_coords = json.loads(coords_json)

            rospy.loginfo(f"[CLIENT] Received desktop coordinates:")
            
            # ✅ 构造 PointCloud 消息
            cloud_msg = PointCloud()
            cloud_msg.header.stamp = rospy.Time.now()
            cloud_msg.header.frame_id = "world"  # 注意要与你的场景 frame_id 一致

            for obj in desktop_coords:
                coord = obj['desktop_coord']
                point = Point32()
                point.x = -0.45-coord[1]-0.11
                point.y = -0.586+coord[0]+0.071
                point.z =  0.65-0.1;  # 可根据具体需求设定高度
                cloud_msg.points.append(point)

            # ✅ 发布到 /dynamic_obstacle_pos
            self.dynamic_obstacle_pub.publish(cloud_msg)
            rospy.loginfo(f"[PUBLISH] Published {len(cloud_msg.points)} dynamic obstacle points.")

            return desktop_coords

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

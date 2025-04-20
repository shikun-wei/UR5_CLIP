#!/usr/bin/env python3

import socket
import json
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header

HOST = '0.0.0.0'
PORT = 4900

class ObstacleReceiver:
    def __init__(self):
        rospy.init_node('obstacle_receiver', anonymous=True)
        self.dynamic_obstacle_pub = rospy.Publisher('/dynamic_obstacle_pos', PointCloud, queue_size=10)
        self.target_pub = rospy.Publisher('/target', PointCloud, queue_size=10)
        self.start_server()

    def start_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            s.settimeout(1.0)  # 设置 accept 超时时间为 1 秒
            rospy.loginfo(f"[SERVER] Listening on {HOST}:{PORT}...")

            while not rospy.is_shutdown():
                try:
                    conn, addr = s.accept()
                except socket.timeout:
                    continue  # 超时后检查 rospy 是否 shutdown
                except Exception as e:
                    rospy.logwarn(f"[SERVER] Accept failed: {e}")
                    continue

                with conn:
                    rospy.loginfo(f"[SERVER] Connected by {addr}")
                    data = b''
                    conn.settimeout(1.0)  # 设置 recv 超时时间

                    while not rospy.is_shutdown():
                        try:
                            chunk = conn.recv(1024)
                            if not chunk:
                                break  # 客户端关闭连接
                            data += chunk
                        except socket.timeout:
                            continue  # 每秒检查一次
                        except Exception as e:
                            rospy.logwarn(f"[SERVER] Recv failed: {e}")
                            break

                    if not data:
                        continue

                    try:
                        decoded = data.decode('utf-8')
                        objects = json.loads(decoded)
                        self.publish_objects(objects)
                    except Exception as e:
                        rospy.logwarn(f"[SERVER] Failed to decode message: {e}")

    def publish_objects(self, objects):
        target_cloud = PointCloud()
        dynamic_cloud = PointCloud()

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        target_cloud.header = header
        dynamic_cloud.header = header

        for obj in objects:
            label = obj.get('label')
            coord = obj.get('desktop_coord', [0, 0])

            point = Point32()
            point.x = coord[0]
            point.y = coord[1]
            point.z = 0.55  # 高度设定为 0.55

            # if label == "Doritos":
            if label == "Perrier soda":
                target_cloud.points.append(point)

            dynamic_cloud.points.append(point)

        if target_cloud.points:
            self.target_pub.publish(target_cloud)
            rospy.loginfo(f"[PUBLISH] Published {len(target_cloud.points)} target point(s).")

        if dynamic_cloud.points:
            self.dynamic_obstacle_pub.publish(dynamic_cloud)
            rospy.loginfo(f"[PUBLISH] Published {len(dynamic_cloud.points)} dynamic obstacle point(s).")

if __name__ == "__main__":
    try:
        ObstacleReceiver()
    except rospy.ROSInterruptException:
        rospy.loginfo("[SERVER] ROS node interrupted, shutting down.")
    except Exception as e:
        rospy.logerr(f"[SERVER] Unexpected error: {e}")

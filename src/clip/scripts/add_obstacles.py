#!/usr/bin/env python3
import rospy
import yaml
import os
import math
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point

# 配置路径
YAML_PATH = '/root/ros_ws/ENSTA_ROB311_Project/src/robot_arm_tools/config/environments/opstacles.yaml'
BOX_SIZE = {'dx': 0.01, 'dy': 0.01, 'dz': 0.15}
MERGE_THRESHOLD = 0.002  # 合并距离小于 2cm 的点

# 全局缓存
snapshots = []
MAX_SNAPSHOTS = 5

def load_existing_yaml(path):
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def save_yaml(data, path):
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

def point_to_obstacle(point, index):
    return {
        f"dynamic_box_{index}": {
            'type': 'box',
            'pose': {
                'x': round(point.x, 4),
                'y': round(point.y, 4),
                # 'z': round(point.z - BOX_SIZE['dz'] / 2.0, 4),
                'z': round(point.z, 4),  # 直接使用点的z坐标作为底部
                'rx': 0.0,
                'ry': 0.0,
                'rz': 0.0
            },
            'size': BOX_SIZE,
            'collisions': True,
            'robot_base_collisions': False
        }
    }

def is_close(p1, p2, threshold=MERGE_THRESHOLD):
    dist = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
    return dist < threshold

def merge_points(all_points):
    merged = []
    for p in all_points:
        if not any(is_close(p, m) for m in merged):
            merged.append(p)
    return merged

def callback(msg):
    global snapshots

    rospy.loginfo("📥 Received %d points", len(msg.points))

    # 坐标变换处理
    transformed_points = []
    for pt in msg.points:
        coord = [pt.x, pt.y]
        new_pt = Point()
        new_pt.x = -0.45 - coord[1] - 0.11
        new_pt.y = -0.586 + coord[0] + 0.071 - 0.121/2
        new_pt.z = pt.z  # 保持 z 不变
        transformed_points.append(new_pt)

    snapshots.append(transformed_points)

    if len(snapshots) >= MAX_SNAPSHOTS:
        rospy.loginfo("🧮 Merging all collected points...")

        all_points = [p for snapshot in snapshots for p in snapshot]
        unique_points = merge_points(all_points)

        rospy.loginfo("🔧 Merged to %d unique points", len(unique_points))

        yaml_data = load_existing_yaml(YAML_PATH)
        yaml_data = {k: v for k, v in yaml_data.items() if not k.startswith("dynamic_box_")}

        for i, pt in enumerate(unique_points):
            new_obj = point_to_obstacle(pt, i)
            yaml_data.update(new_obj)

        save_yaml(yaml_data, YAML_PATH)
        rospy.loginfo("✅ YAML updated with %d dynamic obstacles.", len(unique_points))
        rospy.signal_shutdown("✅ Task complete. Shutting down.")

def main():
    rospy.init_node('dynamic_obstacle_writer_fixed', anonymous=True)
    rospy.Subscriber('/dynamic_obstacle_pos', PointCloud, callback)
    rospy.loginfo("📡 Listening for dynamic obstacle positions (5 frames)...")
    rospy.spin()

if __name__ == '__main__':
    main()

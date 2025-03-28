import numpy as np

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角 (roll, pitch, yaw) 转换为四元数 (x, y, z, w)
    输入角度单位为弧度（radians）
    使用 ZYX 顺序（即先绕 Z 再绕 Y 再绕 X）
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([x, y, z, w])

# 示例：输入欧拉角（弧度）np.deg2rad(60)
roll = 3.14   # 绕 X 轴（Roll）
pitch = 1.57  # 绕 Y 轴（Pitch）
yaw = 0.0    # 绕 Z 轴（Yaw）

q = euler_to_quaternion(roll, pitch, yaw)
formatted_q = [f"{val:.4f}" for val in q]
print("Quaternion [x, y, z, w]:", formatted_q)

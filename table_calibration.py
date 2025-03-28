import cv2
import numpy as np
import json
import os

# Step 1: 加载桌面尺寸
def load_table_config(json_path):
    with open(json_path, 'r') as f:
        config = json.load(f)
    width = config['width']
    height = config['height']
    return width, height

# Step 2: 鼠标回调函数，记录标定点击点
clicked_points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append([x, y])
        print(f"Point {len(clicked_points)}: ({x}, {y})")

# Step 3: 标定函数
def calibrate_table(image_path, config_path):
    img = cv2.imread(image_path)
    width, height = load_table_config(config_path)

    cv2.namedWindow("Select 4 Table Corners", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Select 4 Table Corners", 800, 600)
    cv2.setMouseCallback("Select 4 Table Corners", mouse_callback)

    print("👉 请依次点击桌面的 4 个角（建议顺时针）：")

    while len(clicked_points) < 4:
        img_copy = img.copy()
        for pt in clicked_points:
            cv2.circle(img_copy, tuple(pt), 5, (0, 0, 255), -1)
        cv2.imshow("Select 4 Table Corners", img_copy)
        key = cv2.waitKey(1)
        if key == 27:
            print("⛔ 用户取消操作")
            return None

    cv2.destroyAllWindows()

    src_pts = np.array(clicked_points, dtype=np.float32)
    dst_pts = np.array([
        [0, 0],
        [width, 0],
        [width, height],
        [0, height]
    ], dtype=np.float32)

    H, _ = cv2.findHomography(src_pts, dst_pts)
    print("✅ 单应矩阵 H：\n", H)

    # 保存单应矩阵到 JSON 文件
    h_json_path = "configs/homography_matrix.json"
    os.makedirs(os.path.dirname(h_json_path), exist_ok=True)
    homography_data = {
        "homography_matrix": H.tolist(),
        "table_corner_points": clicked_points  # 这是用户点击的 4 个角点坐标
    }

    with open(h_json_path, 'w') as f:
        json.dump(homography_data, f, indent=2)

    print(f"📁 单应矩阵已保存到：{h_json_path}")

    return H

# Step 4: 使用单应矩阵将图像点映射到桌面坐标
def transform_point(H, u, v):
    pt = np.array([[[u, v]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(pt, H)
    x, y = dst[0][0]
    return x, y

# Step 5: 点击任意图像位置，显示其转换后的桌面坐标
def point_click_loop(H, image_path):
    img = cv2.imread(image_path)
    window_name = "Click to get desk coordinates"

    def click_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            x_desk, y_desk = transform_point(H, x, y)
            print(f"🖱️ 图像点 ({x}, {y}) → 桌面坐标: ({x_desk:.3f} m, {y_desk:.3f} m)")

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)
    cv2.setMouseCallback(window_name, click_callback)

    print("🖱️ 点击图像任意点，查看对应桌面坐标。按 ESC 退出。")

    while True:
        cv2.imshow(window_name, img)
        key = cv2.waitKey(1)
        if key == 27:
            break

    cv2.destroyAllWindows()

# ========== 主程序 ==========
if __name__ == "__main__":
    image_path = "tmp/table.jpg"
    config_path = "configs/table_config.json"

    H = calibrate_table(image_path, config_path)
    if H is not None:
        point_click_loop(H, image_path)

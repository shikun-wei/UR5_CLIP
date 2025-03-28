import torch
from PIL import Image, ImageFilter, ImageDraw, ImageFont
import numpy as np
import cv2
import os
import time
import json
import matplotlib.pyplot as plt
from torchvision import transforms
from EfficientSAM.efficient_sam.efficient_sam import build_efficient_sam
from transformers import CLIPProcessor, CLIPModel

import rospy
from geometry_msgs.msg import Polygon, Point32

clip_model_path = 'D:/Document/Programme/TruckSim/Codes/topological_map/Blip/clip-vit-large-patch14'
sam_model_path = 'D:/Document/Programme/TruckSim/Codes/topological_map/Model/efficient_sam_vitt.pt' # TODO: See README.md to get the download link
image_path = './tmp/table.jpg'
text_queries = [
    "a blue ENSTA Paris metal water bottle with a carabiner", 
    "a blue travel mug with a black lid and handle",
    "white paper",
    "Transparent water cup",
    "Blue and white bottle spray",
]

def load_clip_model():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    processor = CLIPProcessor.from_pretrained(clip_model_path)
    model = CLIPModel.from_pretrained(clip_model_path)
    model.to(device)
    return model, processor, device

clip_model, clip_processor, device = load_clip_model()

def compute_clip_similarity(image, text):
    inputs = clip_processor(text=[text], images=image, return_tensors="pt", padding=True)
    inputs = {k: v.to(device) for k, v in inputs.items()}

    with torch.no_grad():
        image_features = clip_model.get_image_features(pixel_values=inputs["pixel_values"])
        text_features = clip_model.get_text_features(
            input_ids=inputs["input_ids"],
            attention_mask=inputs["attention_mask"]
        )

    similarity = torch.nn.functional.cosine_similarity(image_features, text_features).cpu().item()
    return similarity

# def filter_masks(mask, min_area=20):
#     num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
#     filtered_mask = np.zeros_like(mask)
#     for i in range(1, num_labels):
#         if stats[i, cv2.CC_STAT_AREA] >= min_area:
#             filtered_mask[labels == i] = 1
#     return filtered_mask

def filter_masks(mask, min_area=20):
    mask = mask.astype(np.uint8)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    
    # num_labels 包含背景，所以实际连通块数量是 (num_labels - 1)
    # 如果只有一个连通块，num_labels == 2
    if num_labels == 2:
        # 这时唯一的前景连通块标签是 1
        area = stats[1, cv2.CC_STAT_AREA]
        if area >= min_area:
            # 返回只保留该连通块的二值图
            return (labels == 1).astype(np.uint8)
    
    # 否则（要么没有连通块，要么连通块数目 > 1），都过滤掉
    return np.zeros_like(mask, dtype=np.uint8)


def compute_iou(mask1, mask2):
    intersection = np.logical_and(mask1, mask2).sum()
    union = np.logical_or(mask1, mask2).sum()
    return intersection / union if union > 0 else 0

def crop_masked_region(image, mask):
    mask_np = np.array(mask)
    coords = np.argwhere(mask_np > 0)

    if coords.shape[0] == 0:
        return None

    y_min, x_min = coords.min(axis=0)
    y_max, x_max = coords.max(axis=0)

    cropped_image = image.crop((x_min, y_min, x_max, y_max))
    return cropped_image

sam_model = build_efficient_sam(
    encoder_patch_embed_dim=192, 
    encoder_num_heads=3, 
    checkpoint=sam_model_path
).eval().to(device)


image = Image.open(image_path).convert("RGB")
original_size = image.size
small_size = (512, 512)
image_resized = image.resize(small_size)
image_tensor = transforms.ToTensor()(image_resized).unsqueeze(0).to(device)

output_dir = "./segmentation_results"
os.makedirs(output_dir, exist_ok=True)

def visualize_sift_keypoints(image, keypoints, save_path):
    image_cv = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
    
    for (x, y) in keypoints:
        cv2.circle(image_cv, (x, y), radius=5, color=(0, 255, 0), thickness=-1)
    
    cv2.imwrite(save_path, image_cv)

def visualize_mask_on_image(original_img, mask, save_path):
    img_cv = cv2.cvtColor(np.array(original_img), cv2.COLOR_RGB2BGR)
    mask_cv = mask.astype(np.uint8) * 255
    
    colored_mask = np.zeros_like(img_cv)
    colored_mask[:, :, 2] = mask_cv
    
    alpha = 0.5
    overlay = cv2.addWeighted(img_cv, 1.0, colored_mask, alpha, 0)

    cv2.imwrite(save_path, overlay)

def get_sift_keypoints(image, json_path='configs/homography_matrix.json', original_size=original_size, small_size=small_size, num_points=50):
    # 读取图像灰度
    image_gray = np.array(image.convert("L"))

    # 读取 JSON 文件，获取 table_corner_points
    with open(json_path, 'r') as f:
        data = json.load(f)
    corner_points = np.array(data["table_corner_points"], dtype=np.float32)

    # 根据尺寸缩放 corner_points
    scale_x = small_size[0] / original_size[0]
    scale_y = small_size[1] / original_size[1]
    corner_points[:, 0] *= scale_x
    corner_points[:, 1] *= scale_y
    corner_points = corner_points.astype(np.int32)

    # 创建掩膜，只在四边形区域内部为白色
    mask = np.zeros(image_gray.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [corner_points], 255)

    # 创建 SIFT 检测器
    sift = cv2.SIFT_create()
    keypoints = sift.detect(image_gray, mask)

    # 根据响应值排序，选择前 num_points 个
    keypoints = sorted(keypoints, key=lambda kp: kp.response, reverse=True)[:num_points]

    return [(int(kp.pt[0]), int(kp.pt[1])) for kp in keypoints]


# Extract SIFT keypoints
input_points_list = get_sift_keypoints(image_resized, num_points=80)

sift_keypoints_save_path = os.path.join(output_dir, "sift_keypoints.jpg")
visualize_sift_keypoints(image_resized, input_points_list, sift_keypoints_save_path)
print(f"SIFT Keypoints are saved: {sift_keypoints_save_path}")

text_queries_crop = text_queries
text_queries_position = text_queries

all_points = []
all_labels = []
for point in input_points_list:
    # point: (x, y)
    all_points.append([point])  # shape: (1,2)
    all_labels.append([1])      # shape: (1,)

points_tensor = torch.tensor([all_points], dtype=torch.float, device=device)  # shape (1, N, 1, 2)
labels_tensor = torch.tensor([all_labels], dtype=torch.int, device=device)    # shape (1, N, 1)

with torch.no_grad():
    predicted_logits, predicted_iou = sam_model(image_tensor, points_tensor, labels_tensor)
    # predicted_logits shape: [B=1, 1, N, H, W]
    # predicted_iou    shape: [B=1, N]

print("len(input_points_list) = ", len(input_points_list))
print(f"predicted_logits.shape: {predicted_logits.shape}")

predicted_masks = []
for idx in range(len(input_points_list)):
    logits_i = predicted_logits[0, idx, 0, :, :].cpu().numpy()
    mask_i = (logits_i >= 0).astype(np.uint8)

    mask_i = filter_masks(mask_i, min_area=20)
    if np.sum(mask_i) == 0:
        predicted_masks.append(None)
        continue

    mask_resized = cv2.resize(mask_i, original_size, interpolation=cv2.INTER_NEAREST)
    predicted_masks.append(mask_resized)

print("len(predicted_masks) = ", len(predicted_masks))

unique_masks = []
iou_threshold = 0.9

for i, mask_i in enumerate(predicted_masks):
    if mask_i is None:
        continue

    duplicate_found = False
    for mask_j in unique_masks:
        iou_val = compute_iou(mask_i, mask_j)
        if iou_val > iou_threshold:
            duplicate_found = True
            break

    if not duplicate_found:
        unique_masks.append(mask_i)

print(f"Number of valid partitions retained after de-duplication: {len(unique_masks)}")

crop_images = []
segmented_images = []
blurred_image = image.filter(ImageFilter.GaussianBlur(10))

for mask_resized in unique_masks:
    mask_pil = Image.fromarray((mask_resized * 255).astype(np.uint8)).convert("L")
    segmented_images.append(Image.composite(image, blurred_image, mask_pil))
    cropped_region = crop_masked_region(image, mask_pil)
    crop_images.append(cropped_region)

for idx, segmented_image in enumerate(segmented_images):
    seg_vp_path = os.path.join(output_dir, f"seg_result_vp_{idx}.jpg")
    segmented_image.save(seg_vp_path)
    print(f"Visual Prompting Seg Semantic Segmentation Results for '{idx}' Saved. {seg_vp_path}")

for idx, crop_image in enumerate(crop_images):
    crop_vp_path = os.path.join(output_dir, f"crop_result_vp_{idx}.jpg")
    crop_image.save(crop_vp_path)
    print(f"Visual Prompting Crop Semantic Segmentation Results for '{idx}' Saved. {crop_vp_path}")

time_start = time.time()
top_matches = []

for text_query, text_query_crop in zip(text_queries_position, text_queries_crop):
    best_match = None
    best_similarity = -1
    best_mask = None

    for idx in range(len(segmented_images)):
        similarity_original = compute_clip_similarity(segmented_images[idx], text_query)
        similarity_crop     = compute_clip_similarity(crop_images[idx], text_query_crop)
        final_similarity    = (similarity_original + similarity_crop) / 2
        
        if final_similarity > best_similarity:
            best_similarity = final_similarity
            best_match = segmented_images[idx]
            best_mask = unique_masks[idx]

    print(f"The best similarity of Text query: {text_query}  {best_similarity:.4f}")
    if best_similarity > 0.18:
        top_matches.append((best_match, best_mask, text_query))

time_end = time.time()
print("Computation time =", time_end - time_start)


# # === 新功能：提取底部坐标并映射到桌面坐标 ===
# def get_bottom_center_point(mask):
#     """获取 segmentation mask 最底部的中点"""
#     ys, xs = np.where(mask > 0)
#     if len(ys) == 0:
#         return None
#     max_y = np.max(ys)
#     bottom_xs = xs[ys == max_y]
#     center_x = np.mean(bottom_xs)
#     return np.array([center_x, max_y])

def get_bottom_center_point(mask, num_rows=5, bias=(0.0, 0.0)):
    """
    获取 segmentation mask 最底部 num_rows 行的中间 x 坐标
    """
    ys, xs = np.where(mask > 0)
    if len(ys) == 0:
        return None

    max_y = np.max(ys)
    min_y = max(max_y - num_rows + 1, 0)

    # 找出 y 在 [min_y, max_y] 范围内的像素
    selected_indices = (ys >= min_y) & (ys <= max_y)
    selected_xs = xs[selected_indices]
    selected_ys = ys[selected_indices]

    if len(selected_xs) == 0:
        return None

    center_x = np.mean(selected_xs)
    center_y = np.mean(selected_ys)

    x_bias, y_bias = bias
    return np.array([center_x + x_bias, center_y + y_bias])


def get_bottom_ellipse(mask, num_rows=15):
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None, None

    contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
    max_y = np.max(contour[:, 1])
    min_y = max_y - num_rows
    bottom_points = contour[(contour[:, 1] >= min_y) & (contour[:, 1] <= max_y)]

    if len(bottom_points) < 5:
        return None, None

    ellipse = cv2.fitEllipse(bottom_points)
    center = ellipse[0]
    return np.array(center), ellipse

colors = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (255, 165, 0),
    (128, 0, 128),
    (0, 255, 255),
    (255, 192, 203),
    (165, 42, 42),
    (0, 0, 0),
    (255, 255, 255),
    (192, 192, 192),
    (128, 128, 128),
    (0, 128, 0),
    (0, 0, 128),
    (128, 0, 0),
    (255, 215, 0),
    (75, 0, 130),
    (173, 216, 230),
    (240, 128, 128),
    (152, 251, 152),
    (244, 164, 96),
]

# draw = ImageDraw.Draw(image)
# for idx, (match, mask, label) in enumerate(top_matches):
#     contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     color = colors[idx % len(colors)]
#     for contour in contours:
#         points = [tuple(pt[0]) for pt in contour]
#         draw.line(points + [points[0]], fill=color, width=3)
#     draw.text((10, 30 * idx), label, fill=color, 
#               font=ImageFont.truetype("arial.ttf", 40))

# best_match_path = os.path.join(output_dir, "best_match_labeled.jpg")
# image.save(best_match_path)
# print(f"The consolidated labelling results have been saved to: {best_match_path}")


def apply_homography(H, point):
    """使用 homography 矩阵将图像坐标映射到桌面坐标"""
    point_hom = np.array([point[0], point[1], 1.0])
    transformed = H @ point_hom
    transformed /= transformed[2]
    return transformed[:2]

# === 读取 homography_matrix.json ===
with open('./configs/homography_matrix.json', 'r') as f:
    homography_data = json.load(f)
H = np.array(homography_data["homography_matrix"])

# === 处理 top_matches 中每个物体的 mask ===
desktop_coords = []


# # === 绘制轮廓和底部中心点 ===
# draw = ImageDraw.Draw(image)
# for idx, (match, mask, label) in enumerate(top_matches):
#     # Get center directly
#     # bottom_center = get_bottom_center_point(mask, num_rows=10, bias=(0, 0))  # bias=(1, -9)
    
#     # Ellipse fitting
#     bottom_center, ellipse = get_bottom_ellipse(mask, num_rows=15)
    
#     if bottom_center is None:
#         continue
#     desktop_point = apply_homography(H, bottom_center)
#     coord_info = {
#         "label": label,
#         "desktop_coord": [float(desktop_point[0]), float(desktop_point[1])]
#     }
#     desktop_coords.append(coord_info)
#     print(f"[桌面坐标] {label}: {coord_info['desktop_coord']}")

#     contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     color = colors[idx % len(colors)]

#     # 画轮廓线
#     for contour in contours:
#         points = [tuple(pt[0]) for pt in contour]
#         draw.line(points + [points[0]], fill=color, width=3)

#     # 计算底部中心点（偏移为0，可修改）
#     if bottom_center is not None:
#         x, y = int(bottom_center[0]), int(bottom_center[1])
#         draw.ellipse([(x - 6, y - 6), (x + 6, y + 6)], fill=color, outline="black", width=2)

#     # 左上角写 label
#     draw.text((10, 30 * idx), label, fill=color, font=ImageFont.truetype("arial.ttf", 40))

# Use cv2 to draw
# 初始化图像：确保 image 是 NumPy 格式
# cv_image = np.array(image) if isinstance(image, Image.Image) else image.copy()
cv_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)

for idx, (match, mask, label) in enumerate(top_matches):
    bottom_center, ellipse = get_bottom_ellipse(mask, num_rows=15)
    if bottom_center is None:
        continue

    desktop_point = apply_homography(H, bottom_center)
    desktop_coords.append({
        "label": label,
        "desktop_coord": [float(desktop_point[0]), float(desktop_point[1])]
    })

    color = colors[idx % len(colors)]
    bgr_color = tuple(reversed(color))  # OpenCV 用 BGR

    # 绘制轮廓
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cv_image, contours, -1, bgr_color, 2)

    # 绘制底部圆心
    cx, cy = map(int, bottom_center)
    cv2.circle(cv_image, (cx, cy), radius=6, color=bgr_color, thickness=-1)
    cv2.circle(cv_image, (cx, cy), radius=6, color=(0, 0, 0), thickness=2)  # 黑色边框

    # 绘制 label（左上角）
    label_pos = (10, 40 + 40 * idx)
    cv2.putText(cv_image, label, label_pos, fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1.2, color=bgr_color, thickness=2, lineType=cv2.LINE_AA)

    # 绘制椭圆（如果有）
    if ellipse:
        cv2.ellipse(cv_image, ellipse, bgr_color, 2)



# === 保存到 JSON 文件 ===
with open('./configs/segmentation_desktop_coords.json', 'w') as f:
    json.dump(desktop_coords, f, indent=2)

print("所有物体的桌面坐标已保存到 configs/segmentation_desktop_coords.json")

# best_match_path = os.path.join(output_dir, "best_match_labeled.jpg")
# final_image.save(best_match_path)
# print(f"The consolidated labelling results have been saved to: {best_match_path}")

best_match_path = os.path.join(output_dir, "best_match_labeled.jpg")
cv2.imwrite(best_match_path, cv_image)  # 注意：cv_image 必须是 BGR 格式
print(f"The consolidated labelling results have been saved to: {best_match_path}")


# === 读取桌面尺寸配置 ===
with open('./configs/table_config.json', 'r') as f:
    table_cfg = json.load(f)

table_width_m = table_cfg["width"]   # 单位：米
table_height_m = table_cfg["height"]

# === 设置画布尺寸（像素）===
canvas_width = 1200
canvas_height = int(canvas_width * (table_height_m / table_width_m))

# === 创建空白图像 ===
canvas = Image.new("RGB", (canvas_width + 100, canvas_height + 100), (255, 255, 255))
draw_canvas = ImageDraw.Draw(canvas)

# === 桌面矩形边框偏移量（为留边） ===
margin_x, margin_y = 50, 50

# === 坐标转换比例（桌面米 → 像素）===
scale_x = canvas_width / table_width_m
scale_y = canvas_height / table_height_m

# === 桌面矩形的左上角和右下角像素坐标 ===
rect_top_left = (margin_x, margin_y)
rect_bottom_right = (margin_x + canvas_width, margin_y + canvas_height)

# === 绘制桌面边界矩形 ===
draw_canvas.rectangle([rect_top_left, rect_bottom_right], outline="black", width=5)

# === 绘制点和标签 ===
font = ImageFont.truetype("arial.ttf", 20)
for idx, obj in enumerate(desktop_coords):
    label = obj["label"]
    x_m, y_m = obj["desktop_coord"]  # 单位：米
    color = colors[idx % len(colors)]
    
    # 坐标映射：X 正常，Y 翻转（让左下角是原点）
    x_px = int(x_m * scale_x) + margin_x
    y_px = canvas_height - int(y_m * scale_y) + margin_y

    # 绘制圆点表示位置
    draw_canvas.ellipse([(x_px - 8, y_px - 8), (x_px + 8, y_px + 8)], fill=color, outline=color)

    # 左上角标注文字说明
    draw_canvas.text((10, 20 * idx), label, fill=color, font=font)

# === 保存结果图像 ===
canvas.save("./tmp/desktop_projection_result.jpg")
print("桌面坐标投影图已保存为 tmp/desktop_projection_result.jpg")

def publish_desktop_coords(coords_list):
    rospy.init_node('desktop_obstacle_publisher', anonymous=True)
    pub = rospy.Publisher('/dynamic_obstacles', Polygon, queue_size=10)
    rospy.sleep(1.0)  # 等待连接

    polygon = Polygon()
    for obj in coords_list:
        x, y = obj["desktop_coord"]
        pt = Point32(x=x, y=y, z=0.0)
        polygon.points.append(pt)

    pub.publish(polygon)
    rospy.loginfo("Published %d points to /dynamic_obstacles", len(polygon.points))

try:
    publish_desktop_coords(desktop_coords)
except rospy.ROSInterruptException:
    pass

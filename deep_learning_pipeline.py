import torch
from PIL import Image, ImageDraw, ImageFilter, ImageFont
import numpy as np
import cv2
import os
import time
from torchvision import transforms
from EfficientSAM.efficient_sam.efficient_sam import build_efficient_sam
from transformers import CLIPProcessor, CLIPModel

class DeepLearningProcessor:
    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # 加载 CLIP
        self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-large-patch14")
        self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-large-patch14").to(self.device)

        # 加载 EfficientSAM
        self.sam_model = build_efficient_sam(encoder_patch_embed_dim=192, encoder_num_heads=3, checkpoint='efficient_sam_vitt.pt')
        self.sam_model.eval().to(self.device)

        # self.text_queries = ["a glass", "a bottle", "a cup"]
        # self.text_queries_crop = [text for text in self.text_queries]

        self.text_queries_crop = [
                                "apple", 
                                "blue bottle", 
                                "coca cola",
                                "red cup",
                                "perrier soda "
                            ]
        # self.text_queries_position = [text + " on the table" for text in self.text_queries_crop]
        self.text_queries_position = self.text_queries_crop

        self.output_dir = "./segmentation_results"
        os.makedirs(self.output_dir, exist_ok=True)

    def compute_clip_similarity(self, image, text):
        """ 计算 CLIP 余弦相似度 """
        inputs = self.clip_processor(text=[text], images=image, return_tensors="pt", padding=True)
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            image_features = self.clip_model.get_image_features(pixel_values=inputs["pixel_values"])
            text_features = self.clip_model.get_text_features(input_ids=inputs["input_ids"], attention_mask=inputs["attention_mask"])
        return torch.nn.functional.cosine_similarity(image_features, text_features).cpu().item()

    def compute_iou(self, mask1, mask2):
        intersection = np.logical_and(mask1, mask2).sum()
        union = np.logical_or(mask1, mask2).sum()
        return intersection / union if union > 0 else 0

    def filter_small_masks(self, mask, min_area=200):
        """ 过滤掉面积小于 min_area 的目标 """
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
        filtered_mask = np.zeros_like(mask)
        for i in range(1, num_labels):  # 跳过背景
            if stats[i, cv2.CC_STAT_AREA] >= min_area:
                filtered_mask[labels == i] = 1
        return filtered_mask

    def get_sift_keypoints(self, image, num_points=20):
        """ 使用 SIFT 选取关键点 """
        image = np.array(image.convert("L"))  
        sift = cv2.SIFT_create()
        keypoints = sift.detect(image, None)
        keypoints = sorted(keypoints, key=lambda kp: kp.response, reverse=True)[:num_points]
        return [(int(kp.pt[0]), int(kp.pt[1])) for kp in keypoints]

    def get_grid_keypoints(self, image, num_points=20):
        image = np.array(image.convert("L"))  # 转换为灰度图
        height, width = image.shape
        
        # 选取下半部分区域
        y_start = height // 2  
        
        # 计算网格的行列数
        grid_size = int(np.sqrt(num_points))
        step_x = width // grid_size
        step_y = (height - y_start) // grid_size
        
        keypoints = []
        for i in range(grid_size):
            for j in range(grid_size):
                x = int((i + 0.5) * step_x)
                y = int(y_start + (j + 0.5) * step_y)
                keypoints.append((x, y))
                if len(keypoints) >= num_points:
                    return keypoints
        
        return keypoints

    def crop_masked_region(self, image, mask):
        """在原始图像上裁剪 mask 区域，并返回裁剪后的图像"""
        mask_np = np.array(mask)
        coords = np.argwhere(mask_np > 0)

        if coords.shape[0] == 0:
            return None  # 如果 mask 为空，返回 None

        y_min, x_min = coords.min(axis=0)
        y_max, x_max = coords.max(axis=0)

        cropped_image = image.crop((x_min, y_min, x_max, y_max))
        return cropped_image
    
    def visualize_sift_keypoints(self, image, keypoints, save_path):
        """
        使用 OpenCV 在图像上绘制 SIFT 关键点并保存。
        image: PIL Image
        keypoints: [(x, y), (x, y), ...]
        save_path: 保存路径
        """
        print("len", len(keypoints))
        print("keypoints", keypoints)
        image_cv = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        
        for (x, y) in keypoints:
            cv2.circle(image_cv, (int(x), int(y)), radius=5, color=(0, 255, 0), thickness=-1)
        
        cv2.imwrite(save_path, image_cv)

    def visualize_mask_on_image(self, original_img, mask, save_path):
        """
        将 mask 叠加到原图上并保存，mask 区域用半透明色或边框标记。
        original_img: PIL Image
        mask: np.array, 0/1
        save_path: 保存路径
        """
        img_cv = cv2.cvtColor(np.array(original_img), cv2.COLOR_RGB2BGR)
        mask_cv = mask.astype(np.uint8) * 255
        
        colored_mask = np.zeros_like(img_cv)
        colored_mask[:, :, 2] = mask_cv  # 在红色通道上显示
        
        alpha = 0.5
        overlay = cv2.addWeighted(img_cv, 1.0, colored_mask, alpha, 0)

        cv2.imwrite(save_path, overlay)

    def process_image(self, image_pil):
        """ 处理图像，返回带标注的分割图像，并记录处理时间 """
        time_start = time.time()
        original_size = image_pil.size
        image_resized = image_pil.resize((256, 256))
        image_tensor = transforms.ToTensor()(image_resized).unsqueeze(0).to(self.device)

        keypoints = self.get_grid_keypoints(image_resized, num_points=10)
        self.visualize_sift_keypoints(image_resized, keypoints, os.path.join(self.output_dir, "sift_keypoints.jpg"))
        
        all_points = []
        all_labels = []

        for point in keypoints:
            # point: (x, y)
            # 这里每个 point 都当做 single point prompt，因此 label=1
            all_points.append([point])  # shape: (1,2)
            all_labels.append([1])      # shape: (1,)

        points_tensor = torch.tensor([all_points], dtype=torch.float, device=self.device)  # shape (1, N, 1, 2)
        labels_tensor = torch.tensor([all_labels], dtype=torch.int, device=self.device)    # shape (1, N, 1)

        with torch.no_grad():
            predicted_logits, predicted_iou = self.sam_model(image_tensor, points_tensor, labels_tensor)
            # predicted_logits shape: [B=1, 1, N, H, W]
            # predicted_iou    shape: [B=1, N]

        print("len(keypoints) = ", len(keypoints))

        predicted_masks = []
        for idx in range(len(keypoints)):
            # 取出第 idx 个分割结果
            logits_i = predicted_logits[0, idx, 0, :, :].cpu().numpy()
            mask_i = (logits_i >= 0).astype(np.uint8)

            # 过滤小面积
            mask_i = self.filter_small_masks(mask_i, min_area=20)
            if np.sum(mask_i) == 0:
                predicted_masks.append(None)
                continue

            # 缩放回原图大小
            mask_resized = cv2.resize(mask_i, original_size, interpolation=cv2.INTER_NEAREST)
            predicted_masks.append(mask_resized)

        print("len(predicted_masks) = ", len(predicted_masks))

        unique_masks = []
        iou_threshold = 0.9  # 可根据需求调整

        for i, mask_i in enumerate(predicted_masks):
            if mask_i is None:
                continue

            # 与已保存的所有 mask 对比，若 IoU 高于阈值就视为重复
            duplicate_found = False
            for mask_j in unique_masks:
                iou_val = self.compute_iou(mask_i, mask_j)
                if iou_val > iou_threshold:
                    duplicate_found = True
                    break

            if not duplicate_found:
                unique_masks.append(mask_i)

        print(f"去重后保留的有效分割数量: {len(unique_masks)}")

        # ======================= #
        # (C) 提前缓存所有 segmented_image，避免重复计算
        # ======================= #
        crop_images = []
        segmented_images = []
        blurred_image = image_pil.filter(ImageFilter.GaussianBlur(10))  # 计算一次高斯模糊背景

        for mask_resized in unique_masks:
            mask_pil = Image.fromarray((mask_resized * 255).astype(np.uint8)).convert("L")
            segmented_images.append(Image.composite(image_pil, blurred_image, mask_pil))
            cropped_region = self.crop_masked_region(image_pil, mask_pil)
            crop_images.append(cropped_region)

        # 保存应用高斯模糊进行 Visual Prompting 的分割结果
        for idx, segmented_image in enumerate(segmented_images):
            seg_vp_path = os.path.join(self.output_dir, f"seg_result_vp_{idx}.jpg")
            segmented_image.save(seg_vp_path)
            print(f"'{idx}' 的 Visual Prompting Seg 语义分割结果已保存: {seg_vp_path}")

        for idx, crop_image in enumerate(crop_images):
            crop_vp_path = os.path.join(self.output_dir, f"crop_result_vp_{idx}.jpg")
            crop_image.save(crop_vp_path)
            print(f"'{idx}' 的 Visual Prompting Crop 语义分割结果已保存: {crop_vp_path}")

        # ======================= #
        # 10. 遍历每个文本查询，计算 CLIP 相似度
        # ======================= #
        top_matches = []

        for text_query, text_query_crop in zip(self.text_queries_position, self.text_queries_crop):
            best_match = None
            best_similarity = -1
            best_mask = None

            # 针对去重后的每个唯一分割区域，计算相似度
            for idx in range(len(segmented_images)):
                # # 1) 构造应用高斯模糊的视觉提示图 segmented_image
                # blurred_image = image.filter(ImageFilter.GaussianBlur(10))
                # mask_pil = Image.fromarray((mask_resized * 255).astype(np.uint8)).convert("L")
                # segmented_image = Image.composite(image, blurred_image, mask_pil)

                # 2) 计算相似度
                similarity_original = self.compute_clip_similarity(segmented_images[idx], text_query)
                similarity_crop     = self.compute_clip_similarity(crop_images[idx], text_query_crop)
                final_similarity    = (similarity_original + similarity_crop) / 2
                
                if final_similarity > best_similarity:
                    best_similarity = final_similarity
                    best_match = segmented_images[idx]
                    best_mask = unique_masks[idx]

            print(f"文本 {text_query} 的最好相似度: {best_similarity:.4f}")
            # 设个阈值，用于决定是否真正认为这个文本匹配到了物体
            if best_similarity > 0.195:
                top_matches.append((best_match, best_mask, text_query))

        time_end = time.time()
        print("Computation time =", time_end - time_start)

        # ======================= #
        # 11. 在原图上绘制边界 & 文本，保存整体结果
        # ======================= #
        colors = [
            (255, 0, 0),      # 红色
            (0, 255, 0),      # 绿色
            (0, 0, 255),      # 蓝色
            (255, 255, 0),    # 黄色
            (255, 165, 0),    # 橙色
            (128, 0, 128),    # 紫色
            (0, 255, 255),    # 青色
            (255, 192, 203),  # 粉色
            (165, 42, 42),    # 棕色
            (0, 0, 0),        # 黑色
            (255, 255, 255),  # 白色
            (192, 192, 192),  # 银色
            (128, 128, 128),  # 灰色
            (0, 128, 0),      # 深绿色
            (0, 0, 128),      # 深蓝色
            (128, 0, 0),      # 深红色
            (255, 215, 0),    # 金色
            (75, 0, 130),     # 靛蓝色
            (173, 216, 230),  # 浅蓝色
            (240, 128, 128),  # 浅珊瑚色
            (152, 251, 152),  # 苍绿色
            (244, 164, 96),   # 砂岩色
        ]

        print("image_pil", type(image_pil))
        draw = ImageDraw.Draw(image_pil)
        for idx, (match, mask, label) in enumerate(top_matches):
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            color = colors[idx % len(colors)]
            for contour in contours:
                points = [tuple(pt[0]) for pt in contour]
                draw.line(points + [points[0]], fill=color, width=3)
            # 在图像上写上 label
            draw.text((10, 30 * idx), label, fill=color, font=ImageFont.load_default())

        best_match_path = os.path.join(self.output_dir, "best_match_labeled.jpg")
        image_pil.save(best_match_path)
        print(f"综合标注结果已保存至: {best_match_path}")

        return np.array(image_pil)
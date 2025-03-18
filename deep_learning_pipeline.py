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
        model_path = 'efficient_sam_vitt.pt'
        self.sam_model = build_efficient_sam(encoder_patch_embed_dim=192, encoder_num_heads=3, checkpoint=model_path)
        self.sam_model.eval().to(self.device)

        self.text_queries = ["a glass", "a bottle", "a cup"]
        self.text_queries_crop = [text for text in self.text_queries]
        self.text_queries_position = [text + " on the table" for text in self.text_queries]

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

    def filter_small_masks(self, mask, min_area=1000):
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

    def process_image(self, image_pil):
        """ 处理图像，返回带标注的分割图像，并记录处理时间 """
        start_time = time.time()
        original_size = image_pil.size
        image_resized = image_pil.resize((256, 256))
        image_tensor = transforms.ToTensor()(image_resized).unsqueeze(0).to(self.device)

        keypoints = self.get_sift_keypoints(image_pil, num_points=50)
        draw = ImageDraw.Draw(image_pil)
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]

        try:
            font = ImageFont.truetype("arial.ttf", 40)
        except:
            font = ImageFont.load_default()

        detected_objects = []
        confidence_threshold = 0.2  

        for idx, (text_query, text_query_crop) in enumerate(zip(self.text_queries_position, self.text_queries_crop)):
            best_mask = None
            best_similarity = -1
            best_label = text_query  

            for point in keypoints:
                input_points = torch.tensor([[[point]]]).to(self.device)
                input_labels = torch.ones((1, 1, 1), dtype=torch.int).to(self.device)

                with torch.no_grad():
                    predicted_logits, _ = self.sam_model(image_tensor, input_points, input_labels)
                mask = torch.ge(predicted_logits[0, 0, 0, :, :], 0).cpu().detach().numpy()

                mask = self.filter_small_masks(mask, min_area=200)

                if np.sum(mask) == 0:
                    continue

                mask_resized = cv2.resize(mask.astype(np.uint8), original_size, interpolation=cv2.INTER_NEAREST)

                # 应用高斯模糊
                blurred_image = image_pil.filter(ImageFilter.GaussianBlur(10))
                mask_pil = Image.fromarray((mask_resized * 255).astype(np.uint8)).convert("L")
                segmented_image = Image.composite(image_pil, blurred_image, mask_pil)

                # 计算两种相似度，并取平均
                similarity_original = self.compute_clip_similarity(segmented_image, text_query)
                similarity_crop = self.compute_clip_similarity(image_pil, text_query_crop)
                final_similarity = (similarity_original + similarity_crop) / 2

                if final_similarity > best_similarity:
                    best_similarity = final_similarity
                    best_mask = mask_resized

            if best_mask is not None and best_similarity > confidence_threshold:
                detected_objects.append((best_label, best_similarity))

                contours, _ = cv2.findContours(best_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    points = [tuple(pt[0]) for pt in contour]
                    draw.line(points + [points[0]], fill=colors[idx % len(colors)], width=3)

                draw.text((10, 30 * idx), f"{best_label} ({best_similarity:.2f})", fill=colors[idx % len(colors)], font=font)

        end_time = time.time()
        process_time = end_time - start_time

        print(f"✅ 处理完成 (耗时: {process_time:.2f} 秒)")
        output_path = os.path.join(self.output_dir, f"processed_{int(time.time())}.jpg")
        image_pil.save(output_path)
        print(f"✅ 处理后的图像已保存至 {output_path}")

        return np.array(image_pil)

import socket
import struct
import numpy as np
import cv2
import os
import time
import signal
from deep_learning_pipeline import DeepLearningProcessor
from PIL import Image

class DeepLearningServer:
    def __init__(self, host="0.0.0.0", port=9999):
        self.host = host
        self.port = port
        self.processor = DeepLearningProcessor()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # ✅ 允许端口复用
        self.server.bind((host, port))
        self.server.listen(5)
        self.save_dir = "./received_images"
        os.makedirs(self.save_dir, exist_ok=True)
        print(f"[INFO] Deep Learning Server is running on {host}:{port}")

        # ✅ 处理 Ctrl + C (SIGINT) 和 终止信号 (SIGTERM)
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

    def receive_image(self, conn):
        """ 接收 RGB 和 深度图 """
        def recv_exactly(conn, size):
            """ 保证按指定大小接收数据 """
            data = b""
            while len(data) < size:
                packet = conn.recv(size - len(data))
                if not packet:
                    return None
                data += packet
            return data

        try:
            # 1️⃣ 接收 RGB 图像
            rgb_size = struct.unpack(">L", conn.recv(4))[0]
            rgb_data = recv_exactly(conn, rgb_size)
            rgb_image_np = cv2.imdecode(np.frombuffer(rgb_data, dtype=np.uint8), cv2.IMREAD_COLOR)

            # 2️⃣ 接收 深度图
            depth_size = struct.unpack(">L", conn.recv(4))[0]
            depth_data = recv_exactly(conn, depth_size)
            depth_image_np = cv2.imdecode(np.frombuffer(depth_data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)

            if rgb_image_np is None or depth_image_np is None:
                print("[ERROR] Failed to receive images!")
                return None, None

            return rgb_image_np, depth_image_np
        except Exception as e:
            print(f"[ERROR] Exception in receive_image: {e}")
            return None, None

    def save_images(self, rgb_image, depth_image):
        """ 保存 RGB 图像和深度图 """
        timestamp = time.strftime("%Y%m%d-%H%M%S")

        # ✅ 保存 RGB 图像
        rgb_path = os.path.join(self.save_dir, f"rgb_{timestamp}.jpg")
        cv2.imwrite(rgb_path, rgb_image)
        print(f"[INFO] RGB image saved: {rgb_path}")

        # ✅ 确保深度数据是 16-bit
        if depth_image.dtype != np.uint16:
            print(f"[WARNING] Depth image dtype is {depth_image.dtype}, converting to uint16.")
            depth_image = depth_image.astype(np.uint16)

        # ✅ 保存原始深度图
        depth_path = os.path.join(self.save_dir, f"depth_{timestamp}.png")
        cv2.imwrite(depth_path, depth_image)
        print(f"[INFO] Depth image saved: {depth_path}")

        # ✅ 归一化深度图用于可视化
        min_depth = np.min(depth_image)
        max_depth = np.max(depth_image)
        print(f"[INFO] Depth Image Min: {min_depth}, Max: {max_depth}")

        if max_depth > 0:
            depth_vis = ((depth_image - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)
        else:
            depth_vis = np.zeros_like(depth_image, dtype=np.uint8)

        depth_vis_path = os.path.join(self.save_dir, f"depth_vis_{timestamp}.jpg")
        cv2.imwrite(depth_vis_path, depth_vis)
        print(f"[INFO] Depth visualization saved: {depth_vis_path}")

        return rgb_path, depth_path, depth_vis_path

    def process_rgb_image(self, rgb_image):
        """ 处理 RGB 图像 """
        try:
            print("\n📡 [SERVER] Processing RGB image...")
            start_time = time.time()

            # OpenCV 格式转换为 PIL 以供 DeepLearningProcessor 处理
            rgb_pil = Image.fromarray(cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
            processed_image_np = self.processor.process_image(rgb_pil)

            end_time = time.time()
            process_time = end_time - start_time
            print(f"📡 [SERVER] Processing completed ✅ Time: {process_time:.2f} sec")

            # 显示处理后的图像（可选）
            cv2.imshow("Processed Image", cv2.cvtColor(processed_image_np, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
        except Exception as e:
            print(f"[ERROR] Image processing error: {e}")

    def run(self):
        """ 监听客户端请求 """
        print("[INFO] Server is now running. Press Ctrl+C to stop.")

        try:
            while True:
                conn, addr = self.server.accept()
                print(f"[INFO] Connected by {addr}")

                try:
                    rgb_image, depth_image = self.receive_image(conn)
                    if rgb_image is not None and depth_image is not None:
                        rgb_path, depth_path, depth_vis_path = self.save_images(rgb_image, depth_image)
                        print(f"[INFO] Saved RGB and Depth images for comparison: {rgb_path} & {depth_path}")

                        self.process_rgb_image(rgb_image)  # 处理 RGB 图像
                    conn.sendall(b"DONE")  # ✅ 发送完成信号
                except Exception as e:
                    print(f"[ERROR] Unexpected server error: {e}")
                finally:
                    conn.close()
                    print("[INFO] Connection closed.\n")

        except KeyboardInterrupt:
            self.shutdown(signal.SIGINT, None)

    def shutdown(self, signum, frame):
        """ 服务器优雅退出，释放端口 """
        print("\n[INFO] Shutting down server...")
        self.server.close()
        print("[INFO] Server socket closed. Goodbye!")
        exit(0)

    def __del__(self):
        """ 确保服务器退出时关闭 socket """
        try:
            self.server.close()
            print("[INFO] Server socket closed in destructor.")
        except:
            pass


if __name__ == "__main__":
    server = DeepLearningServer()
    server.run()

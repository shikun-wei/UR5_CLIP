import socket
import struct
import numpy as np
import cv2
import os
import time
from deep_learning_pipeline import DeepLearningProcessor
from PIL import Image

class DeepLearningServer:
    def __init__(self, host="0.0.0.0", port=9999):
        self.processor = DeepLearningProcessor()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((host, port))
        self.server.listen(5)
        print(f"[INFO] Deep Learning Server is running on {host}:{port}")

    def receive_image(self, conn):
        """ 从客户端接收图像数据 """
        try:
            data_size_bytes = conn.recv(4)
            if not data_size_bytes:
                print("[ERROR] No data received!")
                return None
            
            expected_size = struct.unpack(">L", data_size_bytes)[0]
            received_data = b""

            while len(received_data) < expected_size:
                packet = conn.recv(4096)
                if not packet:
                    break
                received_data += packet

            if len(received_data) != expected_size:
                print(f"[ERROR] Incomplete image data! Expected {expected_size}, got {len(received_data)}")
                return None

            image_np = cv2.imdecode(np.frombuffer(received_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

            if image_np is None:
                print("[ERROR] Failed to decode image!")
                return None
            
            print(f"[INFO] Successfully received image of shape: {image_np.shape}")
            return image_np
        except Exception as e:
            print(f"[ERROR] Exception in receive_image: {e}")
            return None

    def send_done_signal(self, conn):
        """ 只发送 'DONE'，通知客户端处理完成 """
        try:
            conn.sendall(b"DONE")
            print("[INFO] Sent DONE signal to client")
        except (BrokenPipeError, ConnectionResetError):
            print("[WARNING] Client disconnected before receiving DONE message.")

    def process_image(self, image_np):
        """ 处理图像并显示，不再重复保存 """
        try:
            print("\n📡 [SERVER] 开始处理图像...")
            start_time = time.time()
            image_pil = Image.fromarray(image_np)
            
            processed_image_np = self.processor.process_image(image_pil)  # 处理图像
            end_time = time.time()
            process_time = end_time - start_time

            # ✅ 仅展示，不保存
            cv2.imshow("Processed Image", cv2.cvtColor(processed_image_np, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)  

            print(f"📡 [SERVER] 处理完成 ✅ 处理时间: {process_time:.2f} 秒")
        except Exception as e:
            print(f"[ERROR] Image processing error: {e}")


    def run(self):
        """ 监听客户端请求，处理图像并返回处理完成信号 """
        while True:
            conn, addr = self.server.accept()
            print(f"[INFO] Connected by {addr}")

            try:
                image_np = self.receive_image(conn)
                if image_np is not None:
                    self.process_image(image_np)  # 处理图像并保存
                self.send_done_signal(conn)  # ✅ 只发送 "DONE"
            except Exception as e:
                print(f"[ERROR] Unexpected server error: {e}")
                self.send_done_signal(conn)  # 发生错误时仍发送 "DONE"
            finally:
                conn.close()
                print("[INFO] Connection closed.\n")


if __name__ == "__main__":
    server = DeepLearningServer()
    server.run()

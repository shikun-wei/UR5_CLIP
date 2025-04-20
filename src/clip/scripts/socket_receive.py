#!/usr/bin/env python3
import socket
import json

HOST = '0.0.0.0'  # 监听本机所有IP地址
PORT = 4900    # 🛠️ 和发送端保持一致

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}...")

        while True:
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                data = b''
                while True:
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    data += chunk
                try:
                    decoded = data.decode('utf-8')
                    objects = json.loads(decoded)
                    print("Received object data:")
                    for obj in objects:
                        label = obj["label"]
                        x, y = obj["desktop_coord"]
                        print(f" - {label}: ({x:.3f}, {y:.3f})")
                except Exception as e:
                    print(f"Failed to decode message: {e}")

if __name__ == "__main__":
    start_server()

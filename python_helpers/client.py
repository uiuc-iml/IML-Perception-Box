import socket
import numpy as np
import cv2
import yaml
import struct
import sys
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Socket Receiver")
parser.add_argument('--config', type=str, required=True, help='Path to the YAML configuration file')
args = parser.parse_args()

# Load the YAML configuration file
with open(args.config, "r") as file:
    config = yaml.safe_load(file)

# Extract frame properties from the YAML file
frame_width = config["Camera"]["cols"]
frame_height = config["Camera"]["rows"]

# Extract camera name
camera_name = config["Camera"]["name"]
print(f"Camera Name: {camera_name}")

# Pose data size (16 doubles)
POSE_SIZE = 16 * 8  # 16 doubles * 8 bytes per double

ip_address = config["SocketPublisher"]["address"]
port = config["SocketPublisher"]["port"]

# Set up the socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((ip_address, port))  # Listening on all interfaces
server_socket.listen(1)

conn, addr = server_socket.accept()
print(f"Connection from {addr} has been established.")

try:
    while True:
        # Step 1: Receive the image size (4 bytes)
        data = b''
        while len(data) < 4:
            packet = conn.recv(4 - len(data))
            if not packet:
                print("Connection closed by sender")
                break
            data += packet
        if len(data) < 4:
            print("Incomplete data received for image size.")
            break
        # Unpack image size
        img_size = struct.unpack('!I', data)[0]  # Network byte order
        print(f"Image size: {img_size} bytes")

        # Step 2: Receive the image data
        image_data = b''
        while len(image_data) < img_size:
            packet = conn.recv(img_size - len(image_data))
            if not packet:
                print("Connection closed by sender")
                break
            image_data += packet
        if len(image_data) < img_size:
            print(f"Incomplete image data received. Expected {img_size} bytes, got {len(image_data)} bytes.")
            continue  # Skip this incomplete frame

        # Step 3: Receive the pose data (128 bytes)
        pose_data = b''
        while len(pose_data) < POSE_SIZE:
            packet = conn.recv(POSE_SIZE - len(pose_data))
            if not packet:
                print("Connection closed by sender")
                break
            pose_data += packet
        if len(pose_data) < POSE_SIZE:
            print(f"Incomplete pose data received. Expected {POSE_SIZE} bytes, got {len(pose_data)} bytes.")
            continue  # Skip this incomplete frame

        # Step 4: Decode the image
        # Since the image is sent as JPEG, we can directly decode it
        frame = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            print("Failed to decode image")
            continue

        # Step 5: Decode the pose data
        # Convert the pose data bytes to a NumPy array of doubles
        pose_array = np.frombuffer(pose_data, dtype='<d')  # Assuming little-endian doubles
        if pose_array.size != 16:
            print(f"Pose data size mismatch. Expected 16 elements, got {pose_array.size}")
            continue
        pose_matrix = pose_array.reshape((4, 4))
        print("Pose Matrix:")
        print(pose_matrix)

        # Step 6: Display the frame
        cv2.imshow("Streamed Frame", frame)

        # Press 'q' to quit the display
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    conn.close()
    server_socket.close()
    cv2.destroyAllWindows()

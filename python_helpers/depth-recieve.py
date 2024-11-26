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

# Set up the socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 5000))  # Listening on all interfaces
server_socket.listen(1)

conn, addr = server_socket.accept()
print(f"Connection from {addr} has been established.")

try:
    while True:
        # Step 1: Receive the color image size (4 bytes)
        data = b''
        while len(data) < 4:
            packet = conn.recv(4 - len(data))
            if not packet:
                print("Connection closed by sender")
                break
            data += packet
        if len(data) < 4:
            print("Incomplete data received for color image size.")
            break
        color_img_size = struct.unpack('!I', data)[0]  # Network byte order
        print(f"Color Image size: {color_img_size} bytes")

        # Step 2: Receive the color image data
        color_image_data = b''
        while len(color_image_data) < color_img_size:
            packet = conn.recv(color_img_size - len(color_image_data))
            if not packet:
                print("Connection closed by sender")
                break
            color_image_data += packet
        if len(color_image_data) < color_img_size:
            print(f"Incomplete color image data received. Expected {color_img_size} bytes, got {len(color_image_data)} bytes.")
            continue  # Skip this incomplete frame

        # Step 3: Receive the depth image size (4 bytes)
        data = b''
        while len(data) < 4:
            packet = conn.recv(4 - len(data))
            if not packet:
                print("Connection closed by sender")
                break
            data += packet
        if len(data) < 4:
            print("Incomplete data received for depth image size.")
            break
        depth_img_size = struct.unpack('!I', data)[0]  # Network byte order
        print(f"Depth Image size: {depth_img_size} bytes")

        # Step 4: Receive the depth image data
        depth_image_data = b''
        while len(depth_image_data) < depth_img_size:
            packet = conn.recv(depth_img_size - len(depth_image_data))
            if not packet:
                print("Connection closed by sender")
                break
            depth_image_data += packet
        if len(depth_image_data) < depth_img_size:
            print(f"Incomplete depth image data received. Expected {depth_img_size} bytes, got {len(depth_image_data)} bytes.")
            continue  # Skip this incomplete frame

        # Step 5: Receive the pose data (128 bytes)
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

        # Step 6: Decode the color image
        color_frame = cv2.imdecode(np.frombuffer(color_image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if color_frame is None:
            print("Failed to decode color image")
            continue

        # Step 7: Decode the depth image
        depth_frame = cv2.imdecode(np.frombuffer(depth_image_data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if depth_frame is None:
            print("Failed to decode depth image")
            continue

        # Step 8: Decode the pose data
        pose_array = np.frombuffer(pose_data, dtype='<d')  # Assuming little-endian doubles
        if pose_array.size != 16:
            print(f"Pose data size mismatch. Expected 16 elements, got {pose_array.size}")
            continue
        pose_matrix = pose_array.reshape((4, 4))
        print("Pose Matrix:")
        print(pose_matrix)

        # Step 9: Display the color frame and depth frame
        cv2.imshow("Color Frame", color_frame)

        # Normalize and display the depth frame for visualization
        depth_visual = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        cv2.imshow("Depth Frame", depth_visual)

        # Press 'q' to quit the display
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    conn.close()
    server_socket.close()
    cv2.destroyAllWindows()

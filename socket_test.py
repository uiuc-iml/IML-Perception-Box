import socket
import json
import numpy as np
import cv2
import struct

# Define server details
HOST = "0.0.0.0"  # Listen on all network interfaces
PORT = 7000       # Must match the C++ socket publisher port

# Function to receive data of a fixed size
def receive_data(conn, size):
    data = b""
    while len(data) < size:
        packet = conn.recv(size - len(data))
        if not packet:
            return None
        data += packet
    return data

# Function to parse incoming data
def parse_socket_data(conn):
    try:
        # Receive the 4-byte header indicating message length
        header = receive_data(conn, 4)
        if not header:
            print("Failed to receive header")
            return None

        msg_length = struct.unpack("!I", header)[0]  # Convert bytes to int

        # Receive the actual message
        message = receive_data(conn, msg_length)
        if not message:
            print("Failed to receive message")
            return None

        # Deserialize JSON
        data = json.loads(message.decode())

        # Extract pose matrix
        pose_matrix = np.array(data["pose"]).reshape(4, 4)
        has_pose = data["has_pose"]

        # Decode images
        color_frame_bytes = bytes(data["color_frame"])
        depth_frame_bytes = bytes(data["depth_frame"])

        # Convert to OpenCV format
        color_frame = cv2.imdecode(np.frombuffer(color_frame_bytes, np.uint8), cv2.IMREAD_COLOR)
        depth_frame = np.frombuffer(depth_frame_bytes, dtype=np.float32).reshape((720, 1280))  # Adjust dimensions

        return pose_matrix, has_pose, color_frame, depth_frame
    except Exception as e:
        print(f"Error parsing socket data: {e}")
        return None

# Main server function
def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.bind((HOST, PORT))
        server_sock.listen(1)
        print(f"Listening on {HOST}:{PORT}...")

        conn, addr = server_sock.accept()
        print(f"Connected by {addr}")

        while True:
            result = parse_socket_data(conn)
            if result:
                pose_matrix, has_pose, color_frame, depth_frame = result

                # Display pose matrix
                print("Pose Matrix:\n", pose_matrix)

                # Show images
                if color_frame is not None:
                    cv2.imshow("Color Frame", color_frame)
                if depth_frame is not None:
                    cv2.imshow("Depth Frame", (depth_frame / np.max(depth_frame) * 255).astype(np.uint8))

                cv2.waitKey(1)

if __name__ == "__main__":
    main()


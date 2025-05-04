import xmlrpc.server
import threading
import time
import queue
import os
import numpy as np
import sys
import open3d as o3d
import open3d.core as o3c
import yaml
import socket
import struct
import cv2
import onnxruntime as ort 
import glob


# parent_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
# sys.path.append(parent_dir)
from reconstruction import Reconstruction
from segmentation_model_loader import MaskformerSegmenter
# from sens_reader import scannet_scene_reader, ScanNetPPReader


class MyServer:
    def __init__(self):
        # Set up the XML-RPC server
        self.server = xmlrpc.server.SimpleXMLRPCServer(('172.16.244.187', 5003))
        self.server.register_introspection_functions()
        self.server.register_function(self.start_mapping)
        self.server.register_function(self.stop_mapping)
        self.server.register_function(self.get_semantic_map)
        self.server.register_function(self.pause_mapping)
        self.server.register_function(self.resume_mapping)
        self.server.register_function(self.get_map_stop_mapping)
        self.server.register_function(self.get_metric_map)
        self.server.register_function(self.get_metric_map_diff_blocks)
        self.server.register_function(self.load_segmentation_model)
        self.server.register_function(self.list_onnx_models)
        self.server.register_function(self.delete_onnx_model)
        self.task_thread = None
        self.queue_thread = None
        self.task_running = False
        self.pause_mapping_flag = True
        self.pause_integration = True
        self.vbg_access_lock = threading.Lock()  
        self.queue_empty = threading.Condition()

        # Dataset parameters

        # Initialize queue and indices
        self.index_queue = 0
        self.index_reconstruction = 0
        self.queue = queue.Queue(maxsize=2000)  # Thread-safe queue
        self.onnx = True
        self.load_config()
        



    def load_segmentation_model(self, name, binary_data):
        try:
            os.makedirs("onnx_models", exist_ok=True)
            model_path = f"onnx_models/{name}.onnx"

            with open(model_path, "wb") as f:
                f.write(binary_data.data)

            # Optional: sanity check load
            ort_session = ort.InferenceSession(model_path, providers=["CUDAExecutionProvider"])
            inputs = [i.name for i in ort_session.get_inputs()]
            outputs = [o.name for o in ort_session.get_outputs()]
            print(f"Model '{name}' loaded. Inputs: {inputs}, Outputs: {outputs}")
            return f"Model '{name}' stored and verified."

        except Exception as e:
            print(f"Failed to load model '{name}': {e}")
            return f"Error: {e}"

    def list_onnx_models(self):
        try:
            os.makedirs("onnx_models", exist_ok=True)
            model_files = glob.glob("onnx_models/*.onnx")
            model_names = [os.path.splitext(os.path.basename(m))[0] for m in model_files]
            return model_names
        except Exception as e:
            return f"Error listing models: {e}"

    def delete_onnx_model(self, name):
        try:
            model_path = f"onnx_models/{name}.onnx"
            if os.path.exists(model_path):
                os.remove(model_path)
                return f"Model '{name}' deleted."
            else:
                return f"Model '{name}' not found."
        except Exception as e:
            return f"Error deleting model '{name}': {e}"

    def load_config(self):
        # Read configuration values from the YAML file
        # config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
        
        # with open(config_path, 'r') as config_file:
        #     config = yaml.safe_load(config_file)

        # Store values from the config into instance variables
        # self.voxel_size = config.get('voxel_size', 0.025)
        # self.trunc = self.voxel_size * config.get('truncation_vsize_multiple', 8)
        # self.res = config.get('res', 8)
        # self.n_labels = config.get('n_labels', 150)
        # self.depth_scale = config.get('depth_scale', 1000.0)
        # self.depth_max = config.get('depth_max', 5.0)
        # self.miu = config.get('miu', 0.001)

        yaml_file_path = '../Yaml-files/zed.yaml'

        # Read the camera YAML file
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)

        # Extract the camera parameters
        self.name = config['Camera']['name']
        fx = config['Camera']['fx']
        fy = config['Camera']['fy']
        cx = config['Camera']['cx']
        cy = config['Camera']['cy']

        self.K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]], dtype=np.float64)
        self.frame_width = config["Camera"]["cols"]
        self.frame_height = config["Camera"]["rows"]
        self.ip_address = config["SocketPublisher"]["address"]
        self.port = config["SocketPublisher"]["port"]


        # get mappnig params
        self.voxel_size = config["Mapping"]["voxel_size"]
        self.trunc = self.voxel_size * config["Mapping"]["truncation_vsize_multiple"]
        self.res = config["Mapping"]["res"]
        self.n_labels = config["Mapping"]["n_labels"]
        self.depth_scale = config["Mapping"]["depth_scale"]
        self.depth_max = config["Mapping"]["depth_max"]
        self.miu = config["Mapping"]["miu"]

        print(f"Configuration Loaded: {config}")


    def start_mapping(self, integrate_semantics=False, color=True, voxel_size=0.05, res=8, initial_num_blocks=17500, onnx_model_name=None, live_stream=False):
        if not self.task_running:
            self.task_running = True
            self.pause_mapping_flag = False
            self.pause_integration = False
            # Start mapping and queue threads
            self.voxel_size = voxel_size
            self.res = res
            self.init_blocks = initial_num_blocks
            self.segmenter = None
            self.semantics = integrate_semantics
            if(integrate_semantics):
                num_labels = self.n_labels
                if(not onnx_model_name):
                    self.segmenter = MaskformerSegmenter()
                else:
                    model_path = f"onnx_models/{onnx_model_name}.onnx"
                    if not os.path.exists(model_path):
                        self.task_running = False
                        return f"Error: ONNX model '{onnx_model_name}' not found."
                    self.onnx = True
                    self.ort_session = ort.InferenceSession(model_path, providers=["CUDAExecutionProvider"])
            else:
                num_labels = None

            self.rec = Reconstruction(
                depth_scale=self.depth_scale,
                depth_max=self.depth_max,
                res=self.res,
                voxel_size=self.voxel_size,
                n_labels=num_labels,
                integrate_color=color,
                init_blocks=self.init_blocks,
                live_stream=live_stream

            )
            self.task_thread = threading.Thread(target=self.mapping, daemon=True)
            self.queue_thread = threading.Thread(target=self.fill_queue_from_socket, daemon=True)
            self.queue_thread.start()
            self.task_thread.start()
            return "Task started"
        else:
            return "Task is already running"

    def stop_mapping(self):
        if self.task_running:
            self.task_running = False
            self.pause_mapping_flag = True
            if self.task_thread.is_alive():
                with self.queue_empty:
                    self.queue_empty.notify()
                self.task_thread.join()  # Wait for mapping thread to finish
                


            # Safely access and delete `rec`
            with self.vbg_access_lock:
                del self.rec
                del self.segmenter
                del self.ort_session
                self.ort_session = None
                self.onnx = False
                self.segmenter = None
                self.rec = None
                self.index_queue = 0
                self.index_reconstruction = 0
                
            o3d.core.cuda.release_cache()
            print("stopping mapping!")
            return 1
        else:
            print("No mapping task was running!")
            return 0

    def get_map_stop_mapping(self):
        # self.pause_integration = True
        ret = self.get_map()
        self.stop_mapping()
        return ret


    def pause_mapping(self):
        self.pause_mapping_flag = True
        return "Mapping paused"

    
    def resume_mapping(self):
        self.pause_mapping_flag = False
        return "Mapping resumed"


    def mapping(self):
        while self.task_running and not self.pause_integration:
            print("Checking queue...")  # Added for debugging
            with self.queue_empty:
                while self.queue.empty():
                    print("waiting for frames")
                    self.queue_empty.wait()
                    if self.queue.empty(): # if queue is still empty, stop+task has notified the condition so break
                        break

                if self.queue.empty():
                    continue
                    
            data_dict = self.queue.get()
            print(f"Processing frame {self.index_reconstruction}...")

            # Ensure safe access to `rec`
            with self.vbg_access_lock:
                self.update_rec(
                    data_dict['color'],
                    data_dict['depth'],
                    data_dict['pose'],
                    data_dict['intrinsics_depth'][:3, :3].astype(np.float64)
                )
            self.index_reconstruction += 1
            self.queue.task_done()


    # def fill_queue(self):
    #     while self.task_running:
    #         # Get a data packet from the dataset
    #         data_dict = self.my_ds[self.index_queue]  # Access dataset by index
    #         with self.queue_empty:
    #             if not self.pause_mapping_flag and not self.queue.full():
    #                 self.queue.put(data_dict)
    #                 self.queue_empty.notify()
                
    #         self.index_queue += 1
    #         time.sleep(0.5)
    #         # waiting for mapping to catchup
    #         if self.index_queue % 100 == 50:
    #             print(f"Added frame {self.index_queue} to the queue.")

    def fill_queue_from_socket(self):
        # host, port = 'localhost', 5003
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.bind((self.ip_address, self.port))  # Listening on the specified interface
            server_socket.listen(1)

            conn, addr = server_socket.accept()
            print(f"Connection from {addr} has been established.")

            while self.task_running:
                
                        # Step 1: Receive the color image size (4 bytes)
                data = b''
                while len(data) < 4:
                    packet = conn.recv(4 - len(data))
                    if not packet:
                        print("Connection closed by sender")
                        sys.exit()
                    data += packet
                if len(data) < 4:
                    print("Incomplete data received for color image size.")
                    break
                # Unpack color image size
                color_img_size = struct.unpack('!I', data)[0]  # Network byte order
                # print(f"Color image size: {color_img_size} bytes")

                # Step 2: Receive the color image data
                color_image_data = b''
                while len(color_image_data) < color_img_size:
                    packet = conn.recv(color_img_size - len(color_image_data))
                    if not packet:
                        print("Connection closed by sender")
                        sys.exit()
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
                        sys.exit()
                    data += packet
                if len(data) < 4:
                    print("Incomplete data received for depth image size.")
                    break
                # Unpack depth image size
                depth_img_size = struct.unpack('!I', data)[0]  # Network byte order
                # print(f"Depth image size: {depth_img_size} bytes")

                # Receive the depth image data if size > 0
                depth_image_data = b''
                if depth_img_size > 0:
                    while len(depth_image_data) < depth_img_size:
                        packet = conn.recv(depth_img_size - len(depth_image_data))
                        if not packet:
                            print("Connection closed by sender")
                            sys.exit()
                        depth_image_data += packet
                    if len(depth_image_data) < depth_img_size:
                        print(f"Incomplete depth image data received. Expected {depth_img_size} bytes, got {len(depth_image_data)} bytes.")
                        continue  # Skip this incomplete frame

                # Step 5: Receive the pose data (128 bytes)
                pose_data = b''
                POSE_SIZE = 128
                while len(pose_data) < POSE_SIZE:
                    packet = conn.recv(POSE_SIZE - len(pose_data))
                    if not packet:
                        print("Connection closed by sender")
                        sys.exit()
                    pose_data += packet
                if len(pose_data) < POSE_SIZE:
                    print(f"Incomplete pose data received. Expected {POSE_SIZE} bytes, got {len(pose_data)} bytes.")
                    continue  # Skip this incomplete frame

                # Step 6: Decode the images
                # Decode color image
                color_frame = cv2.imdecode(np.frombuffer(color_image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if color_frame is None:
                    print("Failed to decode color image")
                    continue

                # Decode depth image if available
                if depth_img_size > 0:
                    
                    if(self.name == "ZED2"):
                        print("ZED2")
                        depth_frame = cv2.imdecode(np.frombuffer(depth_image_data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                        print(depth_frame.shape)
                        print(depth_frame[:,0:20])
                        if depth_frame.dtype != np.float32:
                            depth_frame = depth_frame.astype(np.float32) / 1000.0 

                        depth_o3d = o3d.geometry.Image(depth_frame)
                        color_o3d = o3d.geometry.Image(cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB))

                        # Create RGBD image
                        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                            color_o3d,
                            depth_o3d,
                            depth_scale=1.0,              # Already scaled to meters
                            depth_trunc=5.0,
                            convert_rgb_to_intensity=False
                        )

                        intrinsics = o3d.camera.PinholeCameraIntrinsic(
                            width=1280,
                            height=720,
                            fx=523.2045,
                            fy=523.2045,
                            cx=645.2405,
                            cy=369.2390
                        )

                        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                            rgbd, intrinsic=intrinsics
                        )

                        pcd.transform([[1, 0, 0, 0],
                                    [0, -1, 0, 0],
                                    [0, 0, -1, 0],
                                    [0, 0, 0, 1]])

                        o3d.visualization.draw_geometries([pcd])

                    else:
                        depth_frame = cv2.imdecode(np.frombuffer(depth_image_data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                        depth_frame = depth_frame.astype(np.float32)
                        # print(depth_frame.shape)
                        # print(np.max(depth_frame, axis=1))
    
                    
                    if depth_frame is None:
                        print("Failed to decode depth image")
                        continue
                else:
                    depth_frame = None

                # Decode the pose data
                pose_array = np.frombuffer(pose_data, dtype='<d')  # Assuming little-endian doubles
                if pose_array.size != 16:
                    print(f"Pose data size mismatch. Expected 16 elements, got {pose_array.size}")
                    continue
                pose_matrix = pose_array.reshape((4, 4))
                # print("Pose Matrix:")
                # print(pose_matrix)
                if pose_matrix[3,3] == 0:
                    print(f"Invalid Pose! Skipping frame")
                    continue
                

                # Step 8: Display the frames
                cv2.imshow("Color Frame", color_frame)
                if depth_frame is not None:
                    cv2.imshow("Depth Frame", depth_frame)
                intrinsics = self.K
                # print("Intrensic Matrix:")
                # print(intrinsics)
                # print(type(color_frame))
                # print(type(depth_frame))
                data_dict = {
                'color': color_frame,
                'depth': depth_frame,
                'pose': pose_matrix,
                'intrinsics_depth': intrinsics
                }

                # Add data to the queue
                with self.queue_empty:
                    if not self.pause_mapping_flag and not self.queue.full():
                        self.queue.put(data_dict)
                        self.queue_empty.notify()
                time.sleep(0.2)

        except Exception as e:
            print(f"Error in socket client: {e}")
        finally:
            server_socket.close()
            print("Disconnected from server")
                

    def get_semantic_map(self, map_type="pcd", top_label=True):
        time_b = time.time()
        with self.vbg_access_lock:
            if self.rec is not None:
                print("here")
                pcd, labels = self.rec.extract_point_cloud(return_raw_logits=False)
                points = np.asarray(pcd.points).tolist()
                if top_label:
                    labels = np.argmax(labels, axis=1)
                labels = labels.tolist()
                print(len(points))
                result = {'points': points, 'labels': labels}
            else:
                result = "Reconstruction object is not initialized"
        time_e = time.time()
        print(time_e - time_b)
        return result

    def get_metric_map(self, map_type="pcd"):
        with self.vbg_access_lock:
            if self.rec is not None:
                pcd, colors = self.rec.extract_point_cloud_wcolor(return_raw_logits=False)
                points = np.asarray(pcd.points).tolist()
                colors = colors.tolist()
                result = {'points': points, 'colors': colors}
            else:
                result = "Reconstruction object is not initialized"
        return result
    
    def get_metric_map_diff_blocks(self):
        with self.vbg_access_lock:
            if self.rec is not None:
                pts, cols = self.rec.extract_point_cloud_wcolor_diff_blocks(return_color=True)
            else:
                return "Reconstruction object is not initialized"
        return {
                'points': pts.tolist(),
                'colors': (cols.tolist() if cols is not None else [])
            }



    def update_rec(self, rgb, depth, pose, intrinsics):
        # Perform segmentation and update reconstruction
        semantic_label = None
        if self.semantics:
            if not self.onnx:
                semantic_label = self.segmenter.get_pred_probs(
                rgb, depth, x=depth.shape[0], y=depth.shape[1]
            )
            else:
                print("Here")
                print(rgb.shape)
                print(depth.shape)
                target_shape = (480,640)
                rgb_input = cv2.resize(rgb, (target_shape[1], target_shape[0]))
                depth_input = cv2.resize(depth, (target_shape[1], target_shape[0]))
                depth_input= depth_input.astype(np.float32)
                ort_inputs = {
                "rgb": rgb_input,
                "depth": depth_input
                }
                semantic_label = self.ort_session.run(None, ort_inputs)
                print(semantic_label[0].shape)
                semantic_label = np.eye(21)[semantic_label[0]]

        print(pose)
        self.rec.update_vbg(
            depth,
            intrinsics,
            pose,
            semantic_label=semantic_label,
            color=rgb
        )

    def serve_forever(self):
        # Run the server to handle incoming requests
        self.server.serve_forever()

if __name__ == '__main__':
    server = MyServer()
    server.serve_forever()

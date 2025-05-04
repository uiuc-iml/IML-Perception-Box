import xmlrpc.client
import numpy as np
import open3d as o3d
import cv2
import threading
import time
import torch

class PerceptionBox:
    def __init__(self, address):
        self.server = xmlrpc.client.ServerProxy(address, allow_none=True)
        self._live_streaming_thread = None
        self._visualizer_thread = None
        self._stop_live_streaming = threading.Event()
        self._live_streaming_diff_thread = None
        self._visualizer_diff_thread = None
        self._stop_live_streaming_diff = threading.Event()

    def start_mapping(self, integrate_semantics=False, color=True, voxel_size=0.05, res=8, initial_num_blocks=17500, onnx_model_name=None, live_stream=False):
        self.semantics = integrate_semantics
        self.color = color
        return self.server.start_mapping(integrate_semantics, color, voxel_size, res, initial_num_blocks, onnx_model_name, live_stream)

    def stop_mapping(self):
        return self.server.stop_mapping()

    def pause_mapping(self):
        return self.server.pause_mapping()

    def resume_mapping(self):
        return self.server.resume_mapping()

    def get_metric_map(self):
        return self.server.get_metric_map()
    
    def get_metric_map_diff_blocks(self):
        return self.server.get_metric_map_diff_blocks()

    def get_semantic_map(self, map_type="pcd", top_label=True):
        return self.server.get_semantic_map(map_type, top_label)

    def get_map_stop_mapping(self):
        return self.server.get_map_stop_mapping()

    def send_segmentation_model(self, name, onnx_file_path):
        with open(onnx_file_path, "rb") as f:
            model_binary = xmlrpc.client.Binary(f.read())
        return self.server.load_segmentation_model(name, model_binary)

    def exportmodel_torch_to_onnx(self, model, name, camera_frames_input_size):
        rgb_tensor = torch.randn(1, 3, camera_frames_input_size[0], camera_frames_input_size[1])
        depth_tensor = torch.randn(1, 1, camera_frames_input_size[0], camera_frames_input_size[1])
        torch.onnx.export(
        model,
        (rgb_tensor, depth_tensor),
        f"{name}.onnx",
        input_names=["rgb", "depth"],
        output_names=["segmentation"],
        opset_version=12)
        self.send_segmentation_model(name, f"name.onnx")  


    def list_onnx_models(self):
        return self.server.list_onnx_models()

    def delete_onnx_model(self, name):
        return self.server.delete_onnx_model(name)

    def start_live_streaming(self, refresh_rate=2.0, semantics=False, color=True):
        if not self.semantics and semantics:
            print("Warning: Semantics not being integrated! Streaming metric map instead.")
            semantics = False  
        if not self.color and color:
            print("Warning: Color not being integrated! Streaming without color.")
            color = False  

        if self._live_streaming_thread is not None and self._live_streaming_thread.is_alive():
            print("Live streaming is already running.")
            return

        self._stop_live_streaming.clear()
        self._live_map_data = None  # Shared variable

        def fetch_loop():
            while not self._stop_live_streaming.is_set():
                try:
                    if semantics:
                        map_data = self.get_semantic_map()
                        points = np.array(map_data['points'])
                        labels = np.array(map_data['labels'])

                        rng = np.random.default_rng()
                        label_colors = rng.uniform(0, 1, size=(np.max(labels) + 1, 3))
                        colors = label_colors[labels]
                    else:
                        map_data = self.get_metric_map()
                        points = np.array(map_data['points'])
                        colors = np.array(map_data['colors'])

                    self._live_map_data = (points, colors)

                except Exception as e:
                    print(f"Live streaming fetch error: {e}")

                time.sleep(refresh_rate)

        def visualizer_loop():
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name='PerceptionBox Live Map', width=960, height=720)
            pcd = o3d.geometry.PointCloud()
            added = False

            while not self._stop_live_streaming.is_set():
                if self._live_map_data is not None:
                    points, colors = self._live_map_data
                    pcd.points = o3d.utility.Vector3dVector(points)
                    if colors is not None and colors.size > 0:
                        pcd.colors = o3d.utility.Vector3dVector(colors)

                    if not added:
                        vis.add_geometry(pcd)
                        added = True
                    else:
                        vis.update_geometry(pcd)

                vis.poll_events()
                vis.update_renderer()
                time.sleep(0.03)

            vis.destroy_window()

        # Start threads
        self._live_streaming_thread = threading.Thread(target=fetch_loop, daemon=True)
        self._visualizer_thread = threading.Thread(target=visualizer_loop, daemon=True)
        self._live_streaming_thread.start()
        self._visualizer_thread.start()

    def stop_live_streaming(self):
        if self._live_streaming_thread is not None:
            self._stop_live_streaming.set()
            self._live_streaming_thread.join()
            self._live_streaming_thread = None
            print("Live streaming stopped.")
        if self._visualizer_thread is not None:
            self._visualizer_thread.join()
            self._live_streaming_thread = None

    def start_live_streaming_diff(self, refresh_rate=0.3):
        if self._live_streaming_diff_thread is not None and self._live_streaming_diff_thread.is_alive():
            print("Live diff streaming is already running.")
            return

        self._stop_live_streaming_diff.clear()
        self._merged_pts = []
        self._merged_cols = []

        def add_diff(points, colors):
            self._merged_pts.append(points)
            if colors is not None:
                self._merged_cols.append(colors)

        def clear_diff():
            self._merged_pts.clear()
            self._merged_cols.clear()

        def fetch_diff_loop():
            while not self._stop_live_streaming_diff.is_set():
                try:
                    diff = self.get_metric_map_diff_blocks()
                    points = np.array(diff['points'])
                    colors = np.array(diff['colors']) if diff['colors'] is not None else None
                    print(points.shape, colors)
                    # print(colors)
                    if len(points) > 0:
                        add_diff(points, colors)
                except Exception as e:
                    print(f"[Diff Fetch] Error: {e}")

                time.sleep(refresh_rate)

        def visualizer_diff_loop():
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name='PerceptionBox Live Map (Diff)', width=960, height=720)
            pcd = o3d.geometry.PointCloud()
            # vis.add_geometry(pcd)
            added = False
            while not self._stop_live_streaming_diff.is_set():
                if self._merged_pts:
                    merged_pts = np.concatenate(self._merged_pts, axis=0)
                    # pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(merged_pts)
                    print(merged_pts.shape)
                    if self._merged_cols:
                        merged_cols = np.concatenate(self._merged_cols, axis=0)
                        pcd.colors = o3d.utility.Vector3dVector(merged_cols)

                    # vis.clear_geometries()
                    # vis.add_geometry(pcd)
                    # vis.update_geometry(pcd)
                    # vis.reset_view_point(True)
                    if not added:
                        vis.add_geometry(pcd)
                        added = True
                    else:
                        vis.update_geometry(pcd)



                vis.poll_events()
                vis.update_renderer()
                time.sleep(0.03)

            vis.destroy_window()
            clear_diff()

        self._live_streaming_diff_thread = threading.Thread(target=fetch_diff_loop, daemon=True)
        self._visualizer_diff_thread = threading.Thread(target=visualizer_diff_loop, daemon=True)
        self._live_streaming_diff_thread.start()
        self._visualizer_diff_thread.start()

    def stop_live_streaming_diff(self):
        if self._live_streaming_diff_thread is not None:
            self._stop_live_streaming_diff.set()
            self._live_streaming_diff_thread.join()
            self._live_streaming_diff_thread = None
            print("Live streaming stopped.")
        if self._visualizer_diff_thread is not None:
            self._visualizer_diff_thread.join()
            self._live_streaming_diff_thread = None


    def visualize_point_cloud_color(self, map_data):
        points = np.array(map_data['points'])
        colors = np.array(map_data['colors'])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])

    def visualize_point_cloud_labels(self, map_data, n_labels=21):
        points = np.array(map_data['points'])
        labels = np.array(map_data['labels'])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        rng = np.random.default_rng()
        label_colors = rng.uniform(0, 1, size=(n_labels, 3))
        colors = label_colors[labels]
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])

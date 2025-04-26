import xmlrpc.client
import numpy as np
import open3d as o3d
import cv2
import threading
import time

class PerceptionBox:
    def __init__(self, address):
        self.server = xmlrpc.client.ServerProxy(address)
        self._live_streaming_thread = None
        self._stop_live_streaming = threading.Event()

    def start_mapping(self, integrate_semantics=False, color=True, voxel_size=0.05, res=8, initial_num_blocks=17500, onnx_model_name=None):
        self.semantics = integrate_semantics
        self.color = color
        return self.server.start_mapping(integrate_semantics, color, voxel_size, res, initial_num_blocks, onnx_model_name)

    def stop_mapping(self):
        return self.server.stop_mapping()

    def pause_mapping(self):
        return self.server.pause_mapping()

    def resume_mapping(self):
        return self.server.resume_mapping()

    def get_metric_map(self):
        return self.server.get_metric_map()

    def get_semantic_map(self, map_type="pcd", top_label=True):
        return self.server.get_semantic_map(map_type, top_label)

    def get_map_stop_mapping(self):
        return self.server.get_map_stop_mapping()

    def load_segmentation_model(self, name, onnx_file_path):
        with open(onnx_file_path, "rb") as f:
            model_binary = xmlrpc.client.Binary(f.read())
        return self.server.load_segmentation_model(name, model_binary)

    def list_onnx_models(self):
        return self.server.list_onnx_models()

    def delete_onnx_model(self, name):
        return self.server.delete_onnx_model(name)

def start_live_streaming(self, refresh_rate=1.0, semantics=False, color=True):
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

    def stream_loop():
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='PerceptionBox Live Map', width=960, height=720)
        added = False
        pcd = o3d.geometry.PointCloud()

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

                pcd.points = o3d.utility.Vector3dVector(points)

                if colors is not None and (colors.size) > 0:
                    pcd.colors = o3d.utility.Vector3dVector(colors)


                if not added:
                    vis.add_geometry(pcd)
                    added = True
                else:
                    vis.update_geometry(pcd)

                vis.poll_events()
                vis.update_renderer()

            except Exception as e:
                print(f"Live streaming error: {e}")

            time.sleep(refresh_rate)

        vis.destroy_window()

    self._live_streaming_thread = threading.Thread(target=stream_loop, daemon=True)
    self._live_streaming_thread.start()

    def stop_live_streaming(self):
        if self._live_streaming_thread is not None:
            self._stop_live_streaming.set()
            self._live_streaming_thread.join()
            self._live_streaming_thread = None
            print("Live streaming stopped.")

    @staticmethod
    def visualize_point_cloud_color(map_data):
        points = np.array(map_data['points'])
        colors = np.array(map_data['colors'])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])

    @staticmethod
    def visualize_point_cloud_labels(map_data, n_labels=21):
        points = np.array(map_data['points'])
        labels = np.array(map_data['labels'])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        rng = np.random.default_rng()
        label_colors = rng.uniform(0, 1, size=(n_labels, 3))
        colors = label_colors[labels]
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])

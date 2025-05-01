import xmlrpc.client
import time
import numpy as np
import open3d as o3d
from perception_box import PerceptionBox


def main():
   box = PerceptionBox("http://10.192.251.78:5003")
   # box.load_segmentation_model(name = "finetuned_segformer", onnx_file_path= "onnx_model_transfer/segformer/fine_tuned_segformer.onnx")
   print(box.start_mapping(integrate_semantics=False, color = True, live_stream=True))
   box.start_live_streaming_diff()

   while True:
    key = input("q to stop mapping")
    if key:
        if key.lower() == 'q':
            print("User chose to quit.")
            break
   box.stop_live_streaming_diff()
   
   time_begin = time.time()
   map_data = box.get_metric_map()
   time_end = time.time()

   print(box.stop_mapping())
   print(f"time to transmit the map:{time_end - time_begin}")
   print(type(map_data['points']))
   print(len(map_data['points']))
   points = np.array(map_data['points'])
   colors = np.array(map_data['colors'])
   box.visualize_point_cloud_color(map_data)
   

   # Extract points and labels
   # points = np.array(map_data['points'])
   # labels = np.array(map_data['labels'])
   # print(points.shape)
   # print(labels.shape)
   # labels = np.argmax(labels, axis=1)
   # print(labels[0:100])
   # print(labels[1200:1300])
   # Visualize the point cloud with labels
   n_labels = 21  # Replace with the correct number of labels used in your reconstruction
   # visualize_point_cloud(points, labels, n_labels)
   # visualize_point_cloud_color()

if __name__ == "__main__":
   main()

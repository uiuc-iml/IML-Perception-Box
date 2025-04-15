import xmlrpc.client
import time
import numpy as np
import open3d as o3d

def random_color_mapping(n_labels):
   """Generate random colors for labels."""
   rng = np.random.default_rng()
   return rng.uniform(0, 1, size=(n_labels, 3))

def visualize_point_cloud(points, labels, n_labels):
   """Visualize point cloud with colored labels."""
   # Convert points to Open3D point cloud format
   pcd = o3d.geometry.PointCloud()
   pcd.points = o3d.utility.Vector3dVector(points)

   # Generate random colors for labels
   label_colors = random_color_mapping(n_labels)
   
   # Assign colors to points based on their labels
   colors = label_colors[labels]
   pcd.colors = o3d.utility.Vector3dVector(colors)

   # Visualize
   o3d.visualization.draw_geometries([pcd])

def visualize_point_cloud_color(points, colors):
   pcd = o3d.geometry.PointCloud()
   pcd.points = o3d.utility.Vector3dVector(points)
   pcd.colors = o3d.utility.Vector3dVector(colors)
   o3d.visualization.draw_geometries([pcd])


def main():
   # Connect to the server
   server = xmlrpc.client.ServerProxy("http://10.192.251.78:5003")
   # with open("onnx_model_transfer/ESANET/esanet_with_preproc.onnx", "rb") as f:
   #    model_data = f.read()
   # # send model as a binary using load_segmentation_model()
   # response = server.load_segmentation_model(xmlrpc.client.Binary(model_data))
   # print(response)
   # Start the task
   print(server.start_mapping(False, True))


   # Wait for mapping to process data
   # time.sleep(90)  #
   while True:
    key = input("q to stop mapping")
    if key:
        if key.lower() == 'q':
            print("User chose to quit.")
            break

   # server.pause_task()
   # time.sleep(50)
   # server.resume_task()
   # time.sleep(50)
   # Get the map
   time_begin = time.time()
   map_data = server.get_metric_map()
   time_end = time.time()
   
   
   # Stop the task
   print(server.stop_mapping())
   print(f"time to transmit the map:{time_end - time_begin}")
   print(type(map_data['points']))

   points = np.array(map_data['points'])
   colors = np.array(map_data['colors'])
   visualize_point_cloud_color(points, colors)
   # Extract points and labels
   # points = np.array(map_data['points'])
   # labels = np.array(map_data['labels'])
   # print(points.shape)
   # print(labels.shape)
   # # labels = np.argmax(labels, axis=1)
   # print(labels[0:100])
   # print(labels[1200:1300])
   # Visualize the point cloud with labels
   n_labels = 21  # Replace with the correct number of labels used in your reconstruction
   # visualize_point_cloud(points, labels, n_labels)
   # visualize_point_cloud_color()

if __name__ == "__main__":
   main()

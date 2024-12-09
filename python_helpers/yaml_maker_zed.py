import pyzed.sl as sl
import yaml

# Create a ZED camera object
zed = sl.Camera()

# Initialize the camera with default parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 60
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)

# Get the camera information
camera_info = zed.get_camera_information()
calibration_params = camera_info.camera_configuration.calibration_parameters

# Retrieve camera intrinsics for the left eye and convert to float
focal_left_x = float(calibration_params.left_cam.fx)
focal_left_y = float(calibration_params.left_cam.fy)
cx_left = float(calibration_params.left_cam.cx)
cy_left = float(calibration_params.left_cam.cy)

# Retrieve distortion coefficients for the left eye and convert to float
disto_left = calibration_params.left_cam.disto
k1_left = float(disto_left[0])
k2_left = float(disto_left[1])
p1_left = float(disto_left[2])
p2_left = float(disto_left[3])
k3_left = float(disto_left[4])

# Retrieve camera intrinsics for the right eye and convert to float
focal_right_x = float(calibration_params.right_cam.fx)
focal_right_y = float(calibration_params.right_cam.fy)
cx_right = float(calibration_params.right_cam.cx)
cy_right = float(calibration_params.right_cam.cy)

# Retrieve distortion coefficients for the right eye and convert to float
disto_right = calibration_params.right_cam.disto
k1_right = float(disto_right[0])
k2_right = float(disto_right[1])
p1_right = float(disto_right[2])
p2_right = float(disto_right[3])
k3_right = float(disto_right[4])

# Horizontal field of view
h_fov = float(calibration_params.left_cam.h_fov)

# Stereo calibration: Translation between left and right eye (tx)
tx = float(calibration_params.stereo_transform.get_translation().get()[0])

# Image resolution
cols = int(camera_info.camera_configuration.resolution.width)
rows = int(camera_info.camera_configuration.resolution.height)
fps = float(camera_info.camera_configuration.fps)

# Construct intrinsic matrices
K_left = [
    focal_left_x, 0.0, cx_left,
    0.0, focal_left_y, cy_left,
    0.0, 0.0, 1.0
]

K_right = [
    focal_right_x, 0.0, cx_right,
    0.0, focal_right_y, cy_right,
    0.0, 0.0, 1.0
]

# Identity rotation matrices
R_left = [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0]

R_right = [1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0]

# Translation vectors
T_left = [0.0, 0.0, 0.0]
T_right = [tx, 0.0, 0.0]

# Distortion coefficients
D_left = [k1_left, k2_left, p1_left, p2_left, k3_left]
D_right = [k1_right, k2_right, p1_right, p2_right, k3_right]

# Prepare the data for YAML output
data = {
    'Camera': {
        'name': 'ZED2',
        'setup': 'monocular',  # Assuming stereo setup
        'model': 'perspective',
        'color_order': 'RGB',
        'cols': cols,
        'rows': rows,
        'fps': fps,
        'fx': focal_left_x,
        'fy': focal_left_y,
        'cx': cx_left,
        'cy': cy_left,
        'k1': k1_left,
        'k2': k2_left,
        'p1': p1_left,
        'p2': p2_left,
        'k3': k3_left,
        'h_fov': h_fov,
        'stereo_baseline': tx
    },
    'StereoRectifier': {
        'K_left': K_left,
        'K_right': K_right,
        'R_left': R_left,
        'R_right': R_right,
        'T_left': T_left,
        'T_right': T_right,
        'D_left': D_left,
        'D_right': D_right
    },
    'SocketPublisher': {
        'address': '127.0.0.1',  # Ensure this matches the server IP
        'port': 7000             # Ensure this matches the server port
    },
    'Preprocessing': {
        'min_size': 800
    },
    'Feature': {
        'name': 'default ORB feature extraction setting',
        'max_num_keypoints': 2000,
        'scale_factor': 1.2,
        'num_levels': 8,
        'ini_fast_threshold': 20,
        'min_fast_threshold': 7
    }
}

# Write the data to a YAML file
with open('camera_config.yaml', 'w') as outfile:
    yaml.dump(data, outfile, default_flow_style=False)

print("YAML configuration file 'camera_config.yaml' has been created.")

# Close the camera
zed.close()

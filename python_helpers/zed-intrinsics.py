import pyzed.sl as sl

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

# Retrieve camera intrinsics for the left eye
focal_left_x = calibration_params.left_cam.fx
focal_left_y = calibration_params.left_cam.fy
cx = calibration_params.left_cam.cx
cy = calibration_params.left_cam.cy

# Retrieve distortion coefficients for the left eye
k1 = calibration_params.left_cam.disto[0]
k2 = calibration_params.left_cam.disto[1]
p1 = calibration_params.left_cam.disto[2]
p2 = calibration_params.left_cam.disto[3]
k3 = calibration_params.left_cam.disto[4]


# Horizontal field of view
h_fov = calibration_params.left_cam.h_fov

# Stereo calibration: Translation between left and right eye (tx)
tx = calibration_params.stereo_transform.get_translation().get()[0]

# Display the retrieved parameters
print(f"Focal Length (fx, fy): ({focal_left_x}, {focal_left_y})")
print(f"Principal Point (cx, cy): ({cx}, {cy})")
print(f"Distortion Coefficients (k1, k2, p1, p2, k3): ({k1}, {k2}, {p1}, {p2}, {k3})")
print(f"Horizontal FOV: {h_fov} degrees")
print(f"Translation between left and right eye (tx): {tx} meters")

# Close the camera
zed.close()

import pyzed.sl as sl

# Create a camera object
zed = sl.Camera()

# Initialize camera input parameters
init_params = sl.InitParameters()
status = zed.open(init_params)

if status != sl.ERROR_CODE.SUCCESS:
    print("No ZED cameras connected or failed to open the camera.")
else:
    print("ZED camera opened successfully.")

    # Retrieve camera information
    camera_info = zed.get_camera_information()
    print(f"Camera Model: {camera_info.camera_model}")
    print(f"Serial Number: {camera_info.serial_number}")
    print(f"Firmware Version: {camera_info.camera_configuration.firmware_version}")

    resolution = camera_info.camera_configuration.resolution
    print(f"Resolution: {resolution.width}x{resolution.height}")

    current_fps = zed.get_current_fps()
    print(f"Frame Rate: {current_fps} FPS")

    # Retrieve supported resolutions and framerates
    print("Supported Video Modes:")
    for resolution in [sl.RESOLUTION.HD2K, sl.RESOLUTION.HD1080, sl.RESOLUTION.HD720, sl.RESOLUTION.VGA]:
        for framerate in [15, 30, 60, 100]:
            init_params.camera_resolution = resolution
            init_params.camera_fps = framerate
            status = zed.open(init_params)
            if status == sl.ERROR_CODE.SUCCESS:
                print(f"  Resolution: {resolution.name} at {framerate} FPS")
            zed.close()  # Close the camera after each check

    # Close the camera
    zed.close()

import pyrealsense2 as rs

# Create a context and check for connected devices
context = rs.context()
devices = context.query_devices()
if len(devices) == 0:
    print("No RealSense devices connected.")
else:
    for device in devices:
        print(f"Device: {device.get_info(rs.camera_info.name)}")
        sensors = device.query_sensors()
        for sensor in sensors:
            print(f"  Sensor: {sensor.get_info(rs.camera_info.name)}")
            for profile in sensor.get_stream_profiles():
                try:
                    # Attempt to cast to a video stream profile
                    video_profile = profile.as_video_stream_profile()
                    print(f"    Stream Type: {video_profile.stream_type()} - {video_profile.format()}")
                    print(f"    Resolution: {video_profile.width()}x{video_profile.height()}")
                    print(f"    Framerate: {video_profile.fps()}")
                except Exception:
                    # Skip non-video streams
                    continue

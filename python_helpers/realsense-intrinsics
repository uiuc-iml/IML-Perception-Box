import pyrealsense2 as rs

# Desired stream configuration
desired_resolution = (640, 480)  # Resolution: 1920x1080
desired_format = rs.format.rgb8    # RGB format
desired_fps = 30                   # Framerate (adjust based on your needs)

# Start the pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable the desired RGB stream
config.enable_stream(rs.stream.color, desired_resolution[0], desired_resolution[1], desired_format, desired_fps)

# Start the pipeline
pipeline_profile = pipeline.start(config)

# Get the stream profile and its intrinsics
stream = pipeline_profile.get_stream(rs.stream.color)
intrinsics = stream.as_video_stream_profile().get_intrinsics()

# Print the intrinsics
print(f"Resolution: {intrinsics.width}x{intrinsics.height}")
print(f"Focal Length (fx, fy): ({intrinsics.fx}, {intrinsics.fy})")
print(f"Principal Point (cx, cy): ({intrinsics.ppx}, {intrinsics.ppy})")
print(f"Distortion Model: {intrinsics.model}")
print(f"Distortion Coefficients: {intrinsics.coeffs}")

# Stop the pipeline
pipeline.stop()

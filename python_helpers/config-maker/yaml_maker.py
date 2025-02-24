from flask import Flask, render_template, request, send_file
import pyzed.sl as sl
import pyrealsense2 as rs
import yaml
import os

app = Flask(__name__)

@app.route('/')
def index():
    # Get supported resolutions and frame rates for ZED and RealSense
    zed_resolutions = ["HD2K", "HD1080", "HD720", "VGA"]
    zed_fps_options = [15, 30, 60, 100]

    realsense_resolutions = [(640, 480), (1280, 720), (1920, 1080)]
    realsense_formats = ["RGB8", "YUYV"]
    realsense_fps_options = [15, 30, 60]

    return render_template('index.html', zed_resolutions=zed_resolutions, zed_fps_options=zed_fps_options,
                           realsense_resolutions=realsense_resolutions, realsense_formats=realsense_formats,
                           realsense_fps_options=realsense_fps_options)


@app.route('/generate_yaml', methods=['POST'])
def generate_yaml():
    try:
        camera_type = request.form.get('camera_type')

        # SocketPublisher settings
        socket_address = request.form.get('socket_address', '127.0.0.1')
        socket_port = int(request.form.get('socket_port', 7000))

        if camera_type == 'ZED':
            # ZED Camera Configuration
            resolution = request.form.get('zed_resolution')
            fps = request.form.get('zed_fps')

            if not resolution or not fps:
                return "Error: Resolution and FPS are required fields for ZED."

            zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = getattr(sl.RESOLUTION, resolution)
            init_params.camera_fps = int(fps)
            err = zed.open(init_params)

            if err != sl.ERROR_CODE.SUCCESS:
                return f"Error initializing ZED camera: {err}"

            camera_info = zed.get_camera_information()
            calibration_params = camera_info.camera_configuration.calibration_parameters

            # Generate YAML for ZED
            data = {
                'Camera': {
                    'name': 'ZED2',
                    'setup': 'monocular',
                    'model': 'perspective',
                    'color_order': 'RGB',
                    'cols': int(camera_info.camera_configuration.resolution.width),
                    'rows': int(camera_info.camera_configuration.resolution.height),
                    'fps': float(camera_info.camera_configuration.fps),
                    'fx': float(calibration_params.left_cam.fx),
                    'fy': float(calibration_params.left_cam.fy),
                    'cx': float(calibration_params.left_cam.cx),
                    'cy': float(calibration_params.left_cam.cy),
                    'k1': float(calibration_params.left_cam.disto[0]),
                    'k2': float(calibration_params.left_cam.disto[1]),
                    'p1': float(calibration_params.left_cam.disto[2]),
                    'p2': float(calibration_params.left_cam.disto[3]),
                    'k3': float(calibration_params.left_cam.disto[4]),
                    'h_fov': float(calibration_params.left_cam.h_fov),
                    'stereo_baseline': float(calibration_params.stereo_transform.get_translation().get()[0])
                }
            }
            zed.close()

        elif camera_type == 'RealSense':
            # RealSense Camera Configuration
            resolution = request.form.get('realsense_resolution')
            width, height = map(int, resolution.split('x'))
            fps = int(request.form.get('realsense_fps'))
            format_str = request.form.get('realsense_format')
            desired_format = getattr(rs.format, format_str.lower())

            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, width, height, desired_format, fps)
            pipeline_profile = pipeline.start(config)

            stream = pipeline_profile.get_stream(rs.stream.color)
            intrinsics = stream.as_video_stream_profile().get_intrinsics()

            # Generate YAML for RealSense
            data = {
                'Camera': {
                    'name': 'Intel RealSense D435',
                    'setup': 'monocular',
                    'model': 'perspective',
                    'color_order': 'RGB',
                    'cols': intrinsics.width,
                    'rows': intrinsics.height,
                    'fps': fps,
                    'fx': intrinsics.fx,
                    'fy': intrinsics.fy,
                    'cx': intrinsics.ppx,
                    'cy': intrinsics.ppy,
                    'distortion_model': str(intrinsics.model),
                    'distortion_coefficients': intrinsics.coeffs
                }
            }
            pipeline.stop()

        else:
            return "Error: Invalid camera type selected."

        # Add StereoRectifier and other configurations
        data.update({
            'StereoRectifier': {
                'K_left': [523.0937, 0.0, 645.2420, 0.0, 523.0937, 369.2362, 0.0, 0.0, 1.0],
                'K_right': [523.0937, 0.0, 645.2420, 0.0, 523.0937, 369.2362, 0.0, 0.0, 1.0],
                'R_left': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                'R_right': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                'T_left': [0.0, 0.0, 0.0],
                'T_right': [119.9004, 0.0, 0.0],
                'D_left': [0.0, 0.0, 0.0, 0.0, 0.0],
                'D_right': [0.0, 0.0, 0.0, 0.0, 0.0]
            },
            'SocketPublisher': {
                'address': socket_address,
                'port': socket_port
            },
            'Mapping': {
                'voxel_size': 0.025,
                'res': 8,
                'n_labels': 150,
                'depth_scale': 1000.0,
                'depth_max': 5.0,
                'miu': 0.001,
                'truncation_vsize_multiple': 8
            }
        })

        # Save YAML file
        yaml_file = os.path.join(os.getcwd(), 'camera_config.yaml')
        with open(yaml_file, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

        return send_file(yaml_file, as_attachment=True)

    except Exception as e:
        return f"Error while generating YAML: {e}"


if __name__ == '__main__':
    app.run(debug=True)

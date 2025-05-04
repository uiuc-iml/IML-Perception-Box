from flask import Flask, render_template, request, redirect, url_for, jsonify
import pyzed.sl as sl
import pyrealsense2 as rs
import yaml
import os
import subprocess
import numpy as np
import logging
import time
import threading

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("perception_config.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("perception_config")

app = Flask(__name__)

# Configuration
SAVE_DIR = os.path.expanduser("~/IML-Perception-Box/Yaml-files")
MAPPING_SERVER_PATH = os.path.expanduser("~/IML-Perception-Box/mapping")
SLAM_SERVER_PATH = os.path.expanduser("~/IML-Perception-Box/build")
KIMERA_PATH = os.path.expanduser("~/Kimera-VIO")

# Process management
active_processes = {}

def get_form_list(prefix, keys, defaults, cast_fn=float):
    """Extract and process form values for a group of related fields"""
    values = []
    for k, d in zip(keys, defaults):
        try:
            val = request.form.get(f'{prefix}_{k}')
            values.append(cast_fn(val) if val else d)
        except Exception as e:
            logger.warning(f"Error processing form value {prefix}_{k}: {e}")
            values.append(d)
    return values

def clean_data_for_yaml(data):
    """Clean and convert NumPy or special data types for YAML serialization"""
    if isinstance(data, dict):
        return {k: clean_data_for_yaml(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [clean_data_for_yaml(v) for v in data]
    elif isinstance(data, np.generic):
        return data.item()
    elif isinstance(data, np.ndarray):
        return data.tolist()
    else:
        return data

def stop_process(process_key):
    """Safely stop a running process"""
    if process_key in active_processes and active_processes[process_key].poll() is None:
        try:
            active_processes[process_key].terminate()
            logger.info(f"Process {process_key} terminated")
            return True
        except Exception as e:
            logger.error(f"Error stopping process {process_key}: {e}")
    return False

@app.route('/')
def index():
    """Render the main configuration interface"""
    camera_options = ["ZED", "RealSense"]
    slam_options = ["Stella-VSLAM", "Kimera-VIO"]
    
    # Camera options
    zed_resolutions = ["HD2K", "HD1080", "HD720", "VGA"]
    zed_fps_options = [15, 30, 60, 100]
    realsense_resolutions = [(640, 480), (1280, 720), (1920, 1080)]
    realsense_formats = ["RGB8", "YUYV"]
    realsense_fps_options = [15, 30, 60]

    return render_template('index.html',
                           zed_resolutions=zed_resolutions,
                           zed_fps_options=zed_fps_options,
                           realsense_resolutions=realsense_resolutions,
                           realsense_formats=realsense_formats,
                           realsense_fps_options=realsense_fps_options,
                           camera_options=camera_options,
                           slam_options=slam_options)

@app.route('/generate_yaml', methods=['POST'])
def generate_yaml():
    """Generate YAML configuration files for the selected camera"""
    try:
        if not os.path.exists(SAVE_DIR):
            os.makedirs(SAVE_DIR)

        camera_type = request.form.get('camera_type')
        socket_address = request.form.get('socket_address', '127.0.0.1')
        socket_port = int(request.form.get('socket_port', 7000))
        
        logger.info(f"Generating YAML for {camera_type} camera")

        # Get mapping parameters
        mapping = {
            'voxel_size': float(request.form.get('mapping_voxel_size', 0.025)),
            'res': int(request.form.get('mapping_res', 8)),
            'n_labels': int(request.form.get('mapping_n_labels', 150)),
            'depth_scale': float(request.form.get('mapping_depth_scale', 1000.0)),
            'depth_max': float(request.form.get('mapping_depth_max', 5.0)),
            'miu': float(request.form.get('mapping_miu', 0.001)),
            'truncation_vsize_multiple': int(request.form.get('mapping_truncation_vsize_multiple', 8))
        }

        if camera_type == 'ZED':
            resolution = request.form.get('zed_resolution')
            fps = request.form.get('zed_fps')

            if not resolution or not fps:
                return "Error: Resolution and FPS are required for ZED."

            zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = getattr(sl.RESOLUTION, resolution)
            init_params.camera_fps = int(fps)

            err = zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                logger.error(f"Error initializing ZED camera: {err}")
                return f"Error initializing ZED camera: {err}"

            info = zed.get_camera_information()
            calib = info.camera_configuration.calibration_parameters

            data = {
                'Camera': {
                    'name': 'ZED2',
                    'setup': 'monocular',
                    'model': 'perspective',
                    'color_order': 'RGB',
                    'cols': info.camera_configuration.resolution.width,
                    'rows': info.camera_configuration.resolution.height,
                    'fps': info.camera_configuration.fps,
                    'fx': calib.left_cam.fx,
                    'fy': calib.left_cam.fy,
                    'cx': calib.left_cam.cx,
                    'cy': calib.left_cam.cy,
                    'k1': calib.left_cam.disto[0],
                    'k2': calib.left_cam.disto[1],
                    'p1': calib.left_cam.disto[2],
                    'p2': calib.left_cam.disto[3],
                    'k3': calib.left_cam.disto[4],
                    'h_fov': calib.left_cam.h_fov,
                    'stereo_baseline': calib.stereo_transform.get_translation().get()[0]
                }
            }
            zed.close()
            filename = "zed.yaml"
            logger.info(f"ZED camera configuration fetched successfully")

        elif camera_type == 'RealSense':
            resolution = request.form.get('realsense_resolution')
            width, height = map(int, resolution.split('x'))
            fps = int(request.form.get('realsense_fps'))
            format_str = request.form.get('realsense_format')
            desired_format = getattr(rs.format, format_str.lower())

            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, width, height, desired_format, fps)
            
            try:
                profile = pipeline.start(config)
                intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

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
                        'k1': 0.0,
                        'k2': 0.0,
                        'p1': 0.0,
                        'p2': 0.0,
                        'k3': 0.0,
                        'h_fov': 0.0,
                        'stereo_baseline': 0.0
                    }
                }
                pipeline.stop()
                logger.info(f"RealSense camera configuration fetched successfully")
            except Exception as e:
                logger.error(f"Error accessing RealSense camera: {e}")
                return f"Error accessing RealSense camera: {e}"
                
            filename = "realsense.yaml"

        elif camera_type == 'Custom':
            data = {
                'Camera': {
                    'name': request.form.get('custom_name', 'Custom Camera'),
                    'setup': 'monocular',
                    'model': 'perspective',
                    'color_order': 'RGB',
                    'cols': int(request.form.get('custom_cols', 640)),
                    'rows': int(request.form.get('custom_rows', 480)),
                    'fps': float(request.form.get('custom_fps', 30)),
                    'fx': float(request.form.get('custom_fx', 500)),
                    'fy': float(request.form.get('custom_fy', 500)),
                    'cx': float(request.form.get('custom_cx', 320)),
                    'cy': float(request.form.get('custom_cy', 240)),
                    'k1': 0.0,
                    'k2': 0.0,
                    'p1': 0.0,
                    'p2': 0.0,
                    'k3': 0.0,
                    'h_fov': 0.0,
                    'stereo_baseline': 0.0
                }
            }
            filename = "custom.yaml"
            logger.info("Custom camera configuration created")

        else:
            logger.error(f"Invalid camera type: {camera_type}")
            return "Error: Invalid camera type."

        # Set up stereo rectifier parameters
        K_keys = [str(i) for i in range(9)]
        stereo_rectifier = {
            'K_left': get_form_list('K_left', K_keys, [523.0937, 0.0, 645.2420, 0.0, 523.0937, 369.2362, 0.0, 0.0, 1.0]),
            'K_right': get_form_list('K_right', K_keys, [523.0937, 0.0, 645.2420, 0.0, 523.0937, 369.2362, 0.0, 0.0, 1.0]),
            'R_left': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'R_right': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'T_left': [0.0, 0.0, 0.0],
            'T_right': [119.9004, 0.0, 0.0],
            'D_left': [0.0, 0.0, 0.0, 0.0, 0.0],
            'D_right': [0.0, 0.0, 0.0, 0.0, 0.0]
        }

        # Compile the complete configuration
        data.update({
            'StereoRectifier': stereo_rectifier,
            'SocketPublisher': {'address': socket_address, 'port': socket_port},
            'Mapping': mapping
        })

        # Clean data before dumping to YAML
        data_clean = clean_data_for_yaml(data)

        # Write the YAML file
        yaml_path = os.path.join(SAVE_DIR, filename)
        with open(yaml_path, 'w') as f:
            yaml.dump(data_clean, f, default_flow_style=False, sort_keys=False)
            
        logger.info(f"YAML file generated successfully: {yaml_path}")
        return f"{filename} generated successfully! Configuration saved to {yaml_path}"

    except Exception as e:
        logger.error(f"Error generating YAML: {e}")
        return f"Error generating YAML: {e}"

@app.route('/run_mapping_server', methods=['POST'])
def run_mapping_server():
    """Start the mapping server with the specified configuration"""
    try:
        camera_type = request.form.get('mapping_camera_type')
        server_address = request.form.get('mapping_server_address')
        server_port = request.form.get('mapping_server_port')
        
        if camera_type not in ["zed", "realsense"]:
            logger.error(f"Invalid camera type for mapping server: {camera_type}")
            return "Invalid camera type for mapping server."
            
        yaml_file = f"{camera_type}.yaml"
        yaml_path = f"../Yaml-files/{yaml_file}"
        
        # Stop any existing mapping server
        stop_process('mapping_server')
        
        # Construct the command with the provided YAML file path and server settings
        command = f'cd {MAPPING_SERVER_PATH} && python3 testserver.py --yaml {yaml_path} --address {server_address} --port {server_port}'
        
        # Run in terminal for visual feedback
        terminal_command = f'gnome-terminal -- bash -c "{command}; exec bash"'
        process = subprocess.Popen(terminal_command, shell=True)
        active_processes['mapping_server'] = process
        
        logger.info(f"Mapping server started with {yaml_file}, address={server_address}, port={server_port}")
        return redirect(url_for('index'))
    except Exception as e:
        logger.error(f"Error running mapping server: {e}")
        return f"Error running mapping server: {e}"

@app.route('/run_slam', methods=['POST'])
def run_slam():
    """Start the selected SLAM/VIO system"""
    try:
        slam_type = request.form.get('slam_type')
        
        # Stop any existing SLAM process
        stop_process('slam_process')
        
        if slam_type == 'Stella-VSLAM':
            camera_type = request.form.get('slam_camera_type')
            if camera_type not in ["zed", "realsense"]:
                logger.error(f"Invalid camera type for SLAM: {camera_type}")
                return "Invalid camera type for SLAM."

            yaml_file = f"{camera_type}.yaml"
            command = f'cd {SLAM_SERVER_PATH} && ./run_camera_slam --vocab ../vocab/orb_vocab.fbow --config ../Yaml-files/{yaml_file} --number 4 --viewer pangolin_viewer'
            
            # Run in terminal for visual feedback
            terminal_command = f'gnome-terminal -- bash -c "{command}; exec bash"'
            process = subprocess.Popen(terminal_command, shell=True)
            active_processes['slam_process'] = process
            
            logger.info(f"Stella-VSLAM started with {yaml_file}")
        
        elif slam_type == 'Kimera-VIO':
            dataset_path = request.form.get('kimera_dataset_path', '/home/motion/catkin_ws/src/datasets/zed2_dataset_20250502_145116/')
            command = f'cd {KIMERA_PATH} && bash ./scripts/stereoVIOEuroc.bash -p "{dataset_path}"'
            
            # Run in terminal for visual feedback
            terminal_command = f'gnome-terminal -- bash -c "{command}; exec bash"'
            process = subprocess.Popen(terminal_command, shell=True)
            active_processes['slam_process'] = process
            
            logger.info(f"Kimera-VIO started with dataset path: {dataset_path}")
        
        else:
            logger.error(f"Invalid SLAM type: {slam_type}")
            return "Invalid SLAM type."

        return redirect(url_for('index'))
    except Exception as e:
        logger.error(f"Error running SLAM: {e}")
        return f"Error running SLAM: {e}"

@app.route('/stop_all', methods=['POST'])
def stop_all():
    """Stop all running processes"""
    try:
        stopped_count = 0
        for key in list(active_processes.keys()):
            if stop_process(key):
                stopped_count += 1
        
        logger.info(f"Stopped {stopped_count} active processes")
        return jsonify({"status": "success", "message": f"Stopped {stopped_count} processes"})
    except Exception as e:
        logger.error(f"Error stopping processes: {e}")
        return jsonify({"status": "error", "message": str(e)})

if __name__ == '__main__':
    # Create necessary directories if they don't exist
    os.makedirs(SAVE_DIR, exist_ok=True)
    
    logger.info("Starting Perception System Configuration Tool")
    app.run(debug=True, host='0.0.0.0', port=5000)
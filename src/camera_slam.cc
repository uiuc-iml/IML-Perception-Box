#include <sl/Camera.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <numeric>
#include <algorithm>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/opencv.hpp>

// Stella-VSLAM
#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>
#include <stella_vslam/util/yaml.h>

// Viewers
#ifdef HAVE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
#include <iridescence_viewer/viewer.h>
#endif
#ifdef HAVE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

// ----------------------------
// Structure to hold frame & pose data
// ----------------------------
struct FramePoseData {
    cv::Mat color_frame;
    cv::Mat depth_frame;
    Eigen::Matrix4d pose;
    bool has_pose = false;
};

// ----------------------------
// Function to initialize socket connection
// ----------------------------
int initialize_socket(const std::shared_ptr<stella_vslam::config>& cfg) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return -1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    const std::string ip_address = cfg->yaml_node_["SocketPublisher"]["address"].as<std::string>();
    int port_id = cfg->yaml_node_["SocketPublisher"]["port"].as<int>();

    server_addr.sin_port = htons(port_id);
    server_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        close(sock);
        return -1;
    }

    return sock;
}

// ----------------------------
// Function to initialize ZED camera
// ----------------------------
bool initialize_zed_camera(sl::Camera& zed) {
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init_params.coordinate_units = sl::UNIT::MILLIMETER;

    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error opening ZED camera: " << sl::toString(err) << std::endl;
        return false;
    }

    return true;
}

// ----------------------------
// Function to process SLAM data
// ----------------------------
void process_slam(const std::shared_ptr<stella_vslam::system>& slam,
                  sl::Camera& zed,
                  const cv::Mat& mask,
                  FramePoseData& shared_data,
                  std::mutex& data_mutex,
                  bool& is_not_end) {
    unsigned int num_frame = 0;
    std::vector<double> track_times;

    while (is_not_end) {
        if (slam->terminate_is_requested()) {
            is_not_end = false;
            break;
        }

        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            // Retrieve the RGB image and depth map
            sl::Mat zed_image, zed_depth;
            zed.retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
            zed.retrieveMeasure(zed_depth, sl::MEASURE::DEPTH, sl::MEM::CPU);

            // Convert to OpenCV format
            cv::Mat color_frame = cv::Mat(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::cvtColor(color_frame, color_frame, cv::COLOR_BGRA2BGR);
            cv::Mat depth_frame = cv::Mat(zed_depth.getHeight(), zed_depth.getWidth(), CV_32FC1, zed_depth.getPtr<sl::float1>(sl::MEM::CPU));

            // Track start time
            auto start_time = std::chrono::steady_clock::now();
            double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now().time_since_epoch()).count();

            slam->feed_monocular_frame(color_frame, timestamp, mask);
            auto tracking_state = slam->get_frame_publisher()->get_tracking_state();

            Eigen::Matrix4d cam_pose = Eigen::Matrix4d::Identity();
            bool has_pose = (tracking_state == "Tracking");

            if (has_pose) {
                cam_pose = slam->get_map_publisher()->get_current_cam_pose();
            }

            // Compute tracking time
            auto end_time = std::chrono::steady_clock::now();
            track_times.push_back(std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count());

            // Store data in shared memory
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                color_frame.copyTo(shared_data.color_frame);
                depth_frame.copyTo(shared_data.depth_frame);
                shared_data.pose = cam_pose;
                shared_data.has_pose = has_pose;
            }

            ++num_frame;
        }
    }

    // Shutdown SLAM
    while (slam->loop_BA_is_running()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
    slam->shutdown();
}

// ----------------------------
// Function to manage viewer
// ----------------------------
void run_viewer(const std::shared_ptr<stella_vslam::system>& slam, const std::string& viewer_string) {
#ifdef HAVE_PANGOLIN_VIEWER
    if (viewer_string == "pangolin_viewer") {
        std::shared_ptr<pangolin_viewer::viewer> pangolin_viewer = std::make_shared<pangolin_viewer::viewer>(
            slam->get_frame_publisher(), slam->get_map_publisher());
        pangolin_viewer->run();
    }
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
    if (viewer_string == "iridescence_viewer") {
        std::shared_ptr<iridescence_viewer::viewer> iridescence_viewer = std::make_shared<iridescence_viewer::viewer>(
            slam->get_frame_publisher(), slam->get_map_publisher());
        iridescence_viewer->run();
    }
#endif
#ifdef HAVE_SOCKET_PUBLISHER
    if (viewer_string == "socket_publisher") {
        std::shared_ptr<socket_publisher::publisher> socket_publisher = std::make_shared<socket_publisher::publisher>(
            slam->get_frame_publisher(), slam->get_map_publisher());
        socket_publisher->run();
    }
#endif
}

// ----------------------------
// Main function
// ----------------------------
int mono_tracking_zed_pose_depth(const std::shared_ptr<stella_vslam::system>& slam,
                                 const std::shared_ptr<stella_vslam::config>& cfg,
                                 const unsigned int cam_num,
                                 const std::string& mask_img_path,
                                 const float scale,
                                 const std::string& map_db_path,
                                 const std::string& viewer_string) {
    cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    int sock = initialize_socket(cfg);
    if (sock == -1) {
        slam->shutdown();
        return EXIT_FAILURE;
    }

    sl::Camera zed;
    if (!initialize_zed_camera(zed)) {
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    FramePoseData shared_data;
    std::mutex data_mutex;
    bool is_not_end = true;

    std::thread slam_thread(process_slam, slam, std::ref(zed), std::ref(mask), std::ref(shared_data), std::ref(data_mutex), std::ref(is_not_end));
    std::thread viewer_thread(run_viewer, slam, viewer_string);

    slam_thread.join();
    viewer_thread.join();

    close(sock);
    zed.close();
    return EXIT_SUCCESS;
}


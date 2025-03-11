#ifdef HAVE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
#include "iridescence_viewer/viewer.h"
#endif
#ifdef HAVE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#ifdef HAVE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
#include "iridescence_viewer/viewer.h"
#endif
#ifdef HAVE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/util/stereo_rectifier.h"
#include "stella_vslam/util/yaml.h"
#include "stella_vslam/publish/map_publisher.h"
#include "stella_vslam/publish/frame_publisher.h"

#include <iostream>
#include <chrono>
#include <numeric>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>
//#include <opencv2/highgui.hpp>



#include <arpa/inet.h> // For socket functions
#include <sys/socket.h> 
#include <unistd.h> 

#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem;


#ifdef USE_STACK_TRACE_LOGGER
#include <backward.hpp>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif


// Include necessary headers
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <numeric>
#include <algorithm>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sl/Camera.hpp> // ZED SDK
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// Include SLAM system headers
#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/yaml.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>

// Include viewer headers
#ifdef HAVE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
#include <iridescence_viewer/viewer.h>
#endif
#ifdef HAVE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

// Define the function
int mono_tracking_zed(const std::shared_ptr<stella_vslam::system>& slam,
                           const std::shared_ptr<stella_vslam::config>& cfg,
                           const unsigned int cam_num,
                           const std::string& mask_img_path,
                           const float scale,
                           const std::string& map_db_path,
                           const std::string& viewer_string) {
    // Load the mask image if provided
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // Socket initialization
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        slam->shutdown();
        return EXIT_FAILURE;
    }
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;

    // Retrieve IP address and port from configuration
    const std::string ip_address = cfg->yaml_node_["SocketPublisher"]["address"].as<std::string>();
    int port_id = cfg->yaml_node_["SocketPublisher"]["port"].as<int>();
    server_addr.sin_port = htons(port_id);
    server_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    // Viewer initialization based on the viewer_string parameter
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> pangolin_viewer;
    if (viewer_string == "pangolin_viewer") {
        pangolin_viewer = std::make_shared<pangolin_viewer::viewer>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"),
            slam,
            slam->get_frame_publisher(),
            slam->get_map_publisher());
    }
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
    std::shared_ptr<iridescence_viewer::viewer> iridescence_viewer;
    std::mutex mtx_pause;
    bool is_paused = false;
    std::mutex mtx_terminate;
    bool terminate_is_requested = false;
    std::mutex mtx_step;
    unsigned int step_count = 0;
    if (viewer_string == "iridescence_viewer") {
        iridescence_viewer = std::make_shared<iridescence_viewer::viewer>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "IridescenceViewer"),
            slam->get_frame_publisher(),
            slam->get_map_publisher());
        // Add controls to the viewer
        iridescence_viewer->add_checkbox("Pause", [&is_paused, &mtx_pause](bool check) {
            std::lock_guard<std::mutex> lock(mtx_pause);
            is_paused = check;
        });
        iridescence_viewer->add_button("Step", [&step_count, &mtx_step] {
            std::lock_guard<std::mutex> lock(mtx_step);
            step_count++;
        });
        iridescence_viewer->add_button("Reset", [&slam] {
            slam->request_reset();
        });
        iridescence_viewer->add_button("Save and exit", [&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate, &iridescence_viewer] {
            {
                std::lock_guard<std::mutex> lock1(mtx_pause);
                is_paused = false;
            }
            {
                std::lock_guard<std::mutex> lock2(mtx_terminate);
                terminate_is_requested = true;
            }
            iridescence_viewer->request_terminate();
        });
        iridescence_viewer->add_close_callback([&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate] {
            {
                std::lock_guard<std::mutex> lock1(mtx_pause);
                is_paused = false;
            }
            {
                std::lock_guard<std::mutex> lock2(mtx_terminate);
                terminate_is_requested = true;
            }
        });
    }
#endif
#ifdef HAVE_SOCKET_PUBLISHER
    std::shared_ptr<socket_publisher::publisher> socket_publisher;
    if (viewer_string == "socket_publisher") {
        socket_publisher = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"),
            slam,
            slam->get_frame_publisher(),
            slam->get_map_publisher());
    }
#endif

    // Initialize ZED camera
    sl::Camera zed;

    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720; 
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init_params.coordinate_units = sl::UNIT::MILLIMETER;   

    // Open the ZED camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error opening ZED camera: " << sl::toString(err) << std::endl;
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    struct FramePoseData {
        cv::Mat color_frame;
        cv::Mat depth_frame;
        Eigen::Matrix4d pose;
        bool has_pose = false;
    };
    std::mutex data_mutex;
    FramePoseData shared_data;

    bool is_not_end = true;
    unsigned int num_frame = 0;
    std::vector<double> track_times;

    std::thread slam_thread([&]() {
        while (is_not_end) {
#ifdef HAVE_IRIDESCENCE_VIEWER
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(mtx_pause);
                    if (!is_paused) {
                        break;
                    }
                }
                {
                    std::lock_guard<std::mutex> lock(mtx_step);
                    if (step_count > 0) {
                        step_count--;
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
#endif

#ifdef HAVE_IRIDESCENCE_VIEWER
            {
                std::lock_guard<std::mutex> lock(mtx_terminate);
                if (terminate_is_requested) {
                    is_not_end = false;
                    break;
                }
            }
#else
            if (slam->terminate_is_requested()) {
                is_not_end = false;
                break;
            }
#endif

            if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
                sl::Mat zed_image;
                sl::Mat zed_depth;
                zed.retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
                zed.retrieveMeasure(zed_depth, sl::MEASURE::DEPTH, sl::MEM::CPU);

                cv::Mat rgb_image = cv::Mat(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
                cv::Mat color_frame;
                cv::cvtColor(rgb_image, color_frame, cv::COLOR_BGRA2BGR);

                cv::Mat depth_frame = cv::Mat(zed_depth.getHeight(), zed_depth.getWidth(), CV_32FC1, zed_depth.getPtr<sl::float1>(sl::MEM::CPU));

                // Resize frames if scaling is required
                if (scale != 1.0) {
                    cv::resize(color_frame, color_frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
                    cv::resize(depth_frame, depth_frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
                }

                // Record the start time of SLAM processing
                const auto tp_1 = std::chrono::steady_clock::now();

                // Input the current frame into the SLAM system
                auto now = std::chrono::system_clock::now();
                double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
                slam->feed_monocular_frame(color_frame, timestamp, mask);

                // Get the current tracking state
                auto tracking_state = slam->get_frame_publisher()->get_tracking_state();

                Eigen::Matrix4d cam_pose;
                bool has_pose = false;

                if (tracking_state == "Tracking") {
                    // Get the current camera pose from the map publisher
                    cam_pose = slam->get_map_publisher()->get_current_cam_pose();
                    has_pose = true;

                    // Print the pose matrix
                    std::cout << "Frame " << num_frame << " Pose:\n" << cam_pose << std::endl;

                    // Extract rotation and translation
                    Eigen::Matrix3d rotation = cam_pose.block<3, 3>(0, 0);
                    Eigen::Vector3d translation = cam_pose.block<3, 1>(0, 3);

                    // Convert rotation matrix to Euler angles (yaw, pitch, roll)
                    Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0);
                    Eigen::Vector3d euler_angles_deg = euler_angles * 180.0 / M_PI;

                    // Print translation and rotation
                    std::cout << "Translation (x, y, z): " << translation.transpose() << std::endl;
                    std::cout << "Rotation (yaw, pitch, roll) in degrees: " << euler_angles_deg.transpose() << std::endl;
                } else {
                    std::cout << "Frame " << num_frame << " tracking state: " << tracking_state << std::endl;
                }

                // Record the end time of SLAM processing and calculate tracking time
                const auto tp_2 = std::chrono::steady_clock::now();
                const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
                track_times.push_back(track_time);

                // Update shared data with the latest frames and pose
                {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    color_frame.copyTo(shared_data.color_frame);
                    depth_frame.copyTo(shared_data.depth_frame);
                    if (has_pose) {
                        shared_data.pose = cam_pose;
                        shared_data.has_pose = true;
                    } else {
                        shared_data.has_pose = false;
                    }
                }

                ++num_frame;
            } else {
                // If frame grab fails, wait a bit before retrying
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        // Wait until loop closure and bundle adjustment are finished
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // Shutdown the SLAM system
        slam->shutdown();
    });

    // Thread for sending image and pose data over the socket
    std::thread data_sender_thread([&]() {
        while (is_not_end) {
            cv::Mat color_local, depth_local;
            Eigen::Matrix4d local_pose;
            bool local_has_pose = false;

            // Read shared data
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (!shared_data.color_frame.empty()) {
                    shared_data.color_frame.copyTo(color_local);
                    if (!shared_data.depth_frame.empty()) {
                        shared_data.depth_frame.copyTo(depth_local);
                    }
                    local_has_pose = shared_data.has_pose;
                    if (local_has_pose) {
                        local_pose = shared_data.pose;
                    }
                }
            }

            if (!color_local.empty()) {
                // Serialize color image (JPEG)
                std::vector<uchar> buf;
                cv::imencode(".jpg", color_local, buf);
                int img_size = buf.size();
                int32_t net_img_size = htonl(img_size);

                // Send color image size
                int sent = send(sock, &net_img_size, sizeof(net_img_size), 0);
                if (sent != sizeof(net_img_size)) {
                    perror("Failed to send color image size");
                    break;
                }

                // Send color image data
                int total_sent = 0;
                while (total_sent < img_size) {
                    sent = send(sock, buf.data() + total_sent, img_size - total_sent, 0);
                    if (sent <= 0) {
                        perror("Failed to send color image data");
                        break;
                    }
                    total_sent += sent;
                }
                if (total_sent != img_size) {
                    perror("Failed to send complete color image data");
                    break;
                }

                // Serialize depth image (PNG)
                if (!depth_local.empty()) {
                    // Convert depth image to 16-bit unsigned integer
                    depth_local.convertTo(depth_local, CV_16U);

                    // Encode depth image as PNG
                    std::vector<uchar> depth_buf;
                    cv::imencode(".png", depth_local, depth_buf);
                    int depth_img_size = depth_buf.size();
                    int32_t net_depth_img_size = htonl(depth_img_size);

                    // Send depth image size
                    sent = send(sock, &net_depth_img_size, sizeof(net_depth_img_size), 0);
                    if (sent != sizeof(net_depth_img_size)) {
                        perror("Failed to send depth image size");
                        break;
                    }

                    // Send depth image data
                    total_sent = 0;
                    while (total_sent < depth_img_size) {
                        sent = send(sock, depth_buf.data() + total_sent, depth_img_size - total_sent, 0);
                        if (sent <= 0) {
                            perror("Failed to send depth image data");
                            break;
                        }
                        total_sent += sent;
                    }
                    if (total_sent != depth_img_size) {
                        perror("Failed to send complete depth image data");
                        break;
                    }
                } else {
                    // Send zero size if depth frame is empty
                    int32_t net_depth_img_size = htonl(0);
                    send(sock, &net_depth_img_size, sizeof(net_depth_img_size), 0);
                }

                // Serialize pose data
                double pose_array[16];
                if (local_has_pose) {
                    memcpy(pose_array, local_pose.data(), sizeof(pose_array));
                } else {
                    // Send identity matrix if no pose is available
                    std::fill(std::begin(pose_array), std::end(pose_array), 0.0);
                    // pose_array[0] = pose_array[5] = pose_array[10] = pose_array[15] = 1.0;
                }

                // Send pose data
                total_sent = 0;
                int pose_data_size = sizeof(pose_array);
                while (total_sent < pose_data_size) {
                    sent = send(sock, ((char*)pose_array) + total_sent, pose_data_size - total_sent, 0);
                    if (sent <= 0) {
                        perror("Failed to send pose data");
                        break;
                    }
                    total_sent += sent;
                }
                if (total_sent != pose_data_size) {
                    perror("Failed to send complete pose data");
                    break;
                }
            }

            // Sleep for a while to control the frame rate
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // Approximately 30 FPS
        }
    });

    // Run the viewer in the main thread
    if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        pangolin_viewer->run();
#endif
    }
    if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
        iridescence_viewer->run();
#endif
    }
    if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        socket_publisher->run();
#endif
    }

    // Wait for threads to finish
    data_sender_thread.join();
    slam_thread.join();

    // Close the socket
    close(sock);

    // Close the ZED camera
    zed.close();

    // Output tracking times
    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "Median tracking time: " << track_times.at(track_times.size() / 2) << " [s]" << std::endl;
    std::cout << "Mean tracking time: " << total_track_time / track_times.size() << " [s]" << std::endl;

    // Save the map database if a path is provided
    if (!map_db_path.empty()) {
        if (!slam->save_map_database(map_db_path)) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}





int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    backward::SignalHandling sh;
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto without_vocab = op.add<popl::Switch>("", "without-vocab", "run without vocabulary file");
    auto cam_num = op.add<popl::Value<unsigned int>>("n", "number", "camera number");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto scale = op.add<popl::Value<float>>("s", "scale", "scaling ratio of images", 1.0);
    auto map_db_path_in = op.add<popl::Value<std::string>>("i", "map-db-in", "load a map from this path", "");
    auto map_db_path_out = op.add<popl::Value<std::string>>("o", "map-db-out", "store a map database at this path after slam", "");
    auto log_level = op.add<popl::Value<std::string>>("", "log-level", "log level", "info");
    auto disable_mapping = op.add<popl::Switch>("", "disable-mapping", "disable mapping");
    auto temporal_mapping = op.add<popl::Switch>("", "temporal-mapping", "enable temporal mapping");
    auto viewer = op.add<popl::Value<std::string>>("", "viewer", "viewer [iridescence_viewer, pangolin_viewer, socket_publisher, none]");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!op.unknown_options().empty()) {
        for (const auto& unknown_option : op.unknown_options()) {
            std::cerr << "unknown_options: " << unknown_option << std::endl;
        }
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if ((!vocab_file_path->is_set() && !without_vocab->is_set())
        || !cam_num->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // viewer
    std::string viewer_string;
    if (viewer->is_set()) {
        viewer_string = viewer->value();
        if (viewer_string != "pangolin_viewer"
            && viewer_string != "socket_publisher"
            && viewer_string != "iridescence_viewer"
            && viewer_string != "none") {
            std::cerr << "invalid arguments (--viewer)" << std::endl
                      << std::endl
                      << op << std::endl;
            return EXIT_FAILURE;
        }
#ifndef HAVE_PANGOLIN_VIEWER
        if (viewer_string == "pangolin_viewer") {
            std::cerr << "pangolin_viewer not linked" << std::endl
                      << std::endl
                      << op << std::endl;
            return EXIT_FAILURE;
        }
#endif
#ifndef HAVE_IRIDESCENCE_VIEWER
        if (viewer_string == "iridescence_viewer") {
            std::cerr << "iridescence_viewer not linked" << std::endl
                      << std::endl
                      << op << std::endl;
            return EXIT_FAILURE;
        }
#endif
#ifndef HAVE_SOCKET_PUBLISHER
        if (viewer_string == "socket_publisher") {
            std::cerr << "socket_publisher not linked" << std::endl
                      << std::endl
                      << op << std::endl;
            return EXIT_FAILURE;
        }
#endif
    }
    else {
#ifdef HAVE_IRIDESCENCE_VIEWER
        viewer_string = "iridescence_viewer";
#elif defined(HAVE_PANGOLIN_VIEWER)
        viewer_string = "pangolin_viewer";
#elif defined(HAVE_SOCKET_PUBLISHER)
        viewer_string = "socket_publisher";
#endif
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    spdlog::set_level(spdlog::level::from_str(log_level->value()));

    // load configuration
    std::shared_ptr<stella_vslam::config> cfg;
    try {
        cfg = std::make_shared<stella_vslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // build a slam system
    std::string vocab_file_path_str = (without_vocab->is_set()) ? "" : vocab_file_path->value();
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_file_path_str);
    bool need_initialize = true;
    if (map_db_path_in->is_set()) {
        need_initialize = false;
        const auto path = fs::path(map_db_path_in->value());
        if (path.extension() == ".yaml") {
            YAML::Node node = YAML::LoadFile(path);
            for (const auto& map_path : node["maps"].as<std::vector<std::string>>()) {
                if (!slam->load_map_database(path.parent_path() / map_path)) {
                    return EXIT_FAILURE;
                }
            }
        }
        else {
            if (!slam->load_map_database(path)) {
                return EXIT_FAILURE;
            }
        }
    }
    slam->startup(need_initialize);
    if (disable_mapping->is_set()) {
        slam->disable_mapping_module();
    }
    else if (temporal_mapping->is_set()) {
        slam->enable_temporal_mapping();
        slam->disable_loop_detector();
    }

    // run tracking
    std::string camera_name = cfg->yaml_node_["Camera"]["name"].as<std::string>();

    // Run tracking
    int ret;
    if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        // Select the appropriate tracking function based on the camera name
        if (camera_name == "ZED2") {
            ret = mono_tracking_zed(slam,
                                    cfg,
                                    cam_num->value(),
                                    mask_img_path->value(),
                                    scale->value(),
                                    map_db_path_out->value(),
                                    viewer_string);
        }
        else if (camera_name == "Intel RealSense D435") {

            /*ret = mono_tracking_realsense_pose_depth (slam,
                                          cfg,
                                          cam_num->value(),
                                          mask_img_path->value(),
                                          scale->value(),
                                          map_db_path_out->value(),
                                          viewer_string);
            */
        }
        else {
            // Default to the original mono_tracking function for other cameras
            //Make changes later to be more modular
            /*ret = mono_tracking(slam,
                                cfg,
                                cam_num->value(),
                                mask_img_path->value(),
                                scale->value(),
                                map_db_path_out->value(),
                                viewer_string);
        */
        }
    }
    else if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Stereo) {
        // ret = stereo_tracking_new(slam,
        //                           cfg,
        //                           cam_num->value(),
        //                           mask_img_path->value(),
        //                           scale->value(),
        //                           map_db_path_out->value(),
        //                           viewer_string);
        return 0;
    }
    else {
        throw std::runtime_error("Invalid setup type: " + slam->get_camera()->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return ret;
}

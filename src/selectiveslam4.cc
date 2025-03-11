// main.cpp

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
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
// No optical flow header needed

#include <spdlog/spdlog.h>
#include <popl.hpp>

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

#include <Eigen/Core>
#include <Eigen/Geometry>
// Removed ZED SDK include since we now use a phone camera via OpenCV
// #include <sl/Camera.hpp>

// -----------------------------------------------------------------------------
// Structures and global variables for multi-object tracking

// For interactive box drawing.
cv::Rect currentBox;
bool drawing = false;
cv::Point startPoint;
std::mutex box_mutex;

// Structure representing a tracked object.
struct TrackedObject {
    cv::Rect objectBox;     // Last known location of the object.
    cv::Mat objectTemplate; // Template image extracted from the selected region.
};

// Vector of tracked objects.
std::vector<TrackedObject> trackedObjects;
std::mutex objects_mutex;

// Mouse callback: while dragging, update currentBox; on LBUTTONUP, add a new tracked object.
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    std::lock_guard<std::mutex> lock(box_mutex);
    if (event == cv::EVENT_LBUTTONDOWN) {
        drawing = true;
        startPoint = cv::Point(x, y);
        currentBox = cv::Rect(x, y, 0, 0);
    } else if (event == cv::EVENT_MOUSEMOVE && drawing) {
        currentBox = cv::Rect(startPoint, cv::Point(x, y));
    } else if (event == cv::EVENT_LBUTTONUP) {
        drawing = false;
        currentBox = cv::Rect(startPoint, cv::Point(x, y));
        if (currentBox.area() > 0) {
            std::lock_guard<std::mutex> lockObj(objects_mutex);
            TrackedObject obj;
            obj.objectBox = currentBox;
            obj.objectTemplate.release(); // Start with an empty template.
            trackedObjects.push_back(obj);
            std::cout << "New object selected: " << currentBox << std::endl;
        }
    }
}

// -----------------------------------------------------------------------------
// Modified mono_tracking_phone: full-frame SLAM while multi-object tracking uses template matching.
// This version uses OpenCV's VideoCapture to receive input from a phone (USB webcam mode).
int mono_tracking_phone(const std::shared_ptr<stella_vslam::system>& slam,
                        const std::shared_ptr<stella_vslam::config>& cfg,
                        const unsigned int cam_num,
                        const std::string& mask_img_path,
                        const float scale,
                        const std::string& map_db_path,
                        const std::string& viewer_string) {
    // Optional mask image (unused in this mode).
    const cv::Mat mask_img = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // Socket initialization.
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        slam->shutdown();
        return EXIT_FAILURE;
    }
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
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

    // Viewer initialization.
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
        // NOTE: Removed the extra slam parameter to match the constructor.
        iridescence_viewer = std::make_shared<iridescence_viewer::viewer>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "IridescenceViewer"),
            slam->get_frame_publisher(),
            slam->get_map_publisher());
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

    // --- Initialize phone camera using OpenCV ---
    cv::VideoCapture cap(0);  // Use device index 0 (adjust if needed)
    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream from phone." << std::endl;
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    // Structure for sharing frame and pose data.
    struct FramePoseData {
        cv::Mat color_frame;
        cv::Mat depth_frame;
        Eigen::Matrix4d pose;
        bool has_pose = false;
    };
    std::mutex data_mutex;
    FramePoseData shared_data;

    // Create a window for ROI and object visualization and set the mouse callback.
    cv::namedWindow("ROI Selector");
    cv::setMouseCallback("ROI Selector", mouseCallback, nullptr);

    bool is_not_end = true;
    unsigned int num_frame = 0;
    std::vector<double> track_times;

    // SLAM processing thread.
    std::thread slam_thread([&]() {
        while (is_not_end) {
#ifdef HAVE_IRIDESCENCE_VIEWER
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(mtx_pause);
                    if (!is_paused) { break; }
                }
                {
                    std::lock_guard<std::mutex> lock(mtx_step);
                    if (step_count > 0) { step_count--; break; }
                }
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            {
                std::lock_guard<std::mutex> lock(mtx_terminate);
                if (terminate_is_requested) { is_not_end = false; break; }
            }
#else
            if (slam->terminate_is_requested()) {
                is_not_end = false;
                break;
            }
#endif

            cv::Mat color_frame;
            if (!cap.read(color_frame)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Apply scaling if needed.
            if (scale != 1.0) {
                cv::resize(color_frame, color_frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
            }

            double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            // Feed the captured frame to SLAM.
            slam->feed_monocular_frame(color_frame, timestamp, cv::Mat{});

            auto tracking_state = slam->get_frame_publisher()->get_tracking_state();
            bool has_pose = false;
            Eigen::Matrix4d cam_pose;
            if (tracking_state == "Tracking") {
                cam_pose = slam->get_map_publisher()->get_current_cam_pose();
                has_pose = true;
                std::cout << "Frame " << num_frame << " Pose:\n" << cam_pose << std::endl;
            } else {
                std::cout << "Frame " << num_frame << " tracking state: " << tracking_state << std::endl;
            }

            auto tp_2 = std::chrono::steady_clock::now();
            double track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - std::chrono::steady_clock::now()).count();
            track_times.push_back(track_time);

            {
                std::lock_guard<std::mutex> lock(data_mutex);
                color_frame.copyTo(shared_data.color_frame);
                // Phone camera does not provide a depth frame; use an empty frame.
                shared_data.depth_frame = cv::Mat{};
                shared_data.has_pose = has_pose;
                if (has_pose) { shared_data.pose = cam_pose; }
            }
            ++num_frame;
        }
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
        slam->shutdown();
    });

    // Data sender thread (socket transmission remains unchanged).
    std::thread data_sender_thread([&]() {
        while (is_not_end) {
            cv::Mat color_local, depth_local;
            Eigen::Matrix4d local_pose;
            bool local_has_pose = false;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (!shared_data.color_frame.empty()) {
                    shared_data.color_frame.copyTo(color_local);
                    if (!shared_data.depth_frame.empty())
                        shared_data.depth_frame.copyTo(depth_local);
                    local_has_pose = shared_data.has_pose;
                    if (local_has_pose)
                        local_pose = shared_data.pose;
                }
            }
            if (!color_local.empty()) {
                std::vector<uchar> buf;
                cv::imencode(".jpg", color_local, buf);
                int img_size = buf.size();
                int32_t net_img_size = htonl(img_size);
                int sent = send(sock, &net_img_size, sizeof(net_img_size), 0);
                if (sent != sizeof(net_img_size)) { perror("Failed to send color image size"); break; }
                int total_sent = 0;
                while (total_sent < img_size) {
                    sent = send(sock, buf.data() + total_sent, img_size - total_sent, 0);
                    if (sent <= 0) { perror("Failed to send color image data"); break; }
                    total_sent += sent;
                }
                if (total_sent != img_size) { perror("Failed to send complete color image data"); break; }

                if (!depth_local.empty()) {
                    depth_local.convertTo(depth_local, CV_16U);
                    std::vector<uchar> depth_buf;
                    cv::imencode(".png", depth_local, depth_buf);
                    int depth_img_size = depth_buf.size();
                    int32_t net_depth_img_size = htonl(depth_img_size);
                    sent = send(sock, &net_depth_img_size, sizeof(net_depth_img_size), 0);
                    if (sent != sizeof(net_depth_img_size)) { perror("Failed to send depth image size"); break; }
                    total_sent = 0;
                    while (total_sent < depth_img_size) {
                        sent = send(sock, depth_buf.data() + total_sent, depth_img_size - total_sent, 0);
                        if (sent <= 0) { perror("Failed to send depth image data"); break; }
                        total_sent += sent;
                    }
                    if (total_sent != depth_img_size) { perror("Failed to send complete depth image data"); break; }
                } else {
                    int32_t net_depth_img_size = htonl(0);
                    send(sock, &net_depth_img_size, sizeof(net_depth_img_size), 0);
                }

                double pose_array[16];
                if (local_has_pose) {
                    memcpy(pose_array, local_pose.data(), sizeof(pose_array));
                } else {
                    std::fill(std::begin(pose_array), std::end(pose_array), 0.0);
                }
                total_sent = 0;
                int pose_data_size = sizeof(pose_array);
                while (total_sent < pose_data_size) {
                    sent = send(sock, ((char*)pose_array) + total_sent, pose_data_size - total_sent, 0);
                    if (sent <= 0) { perror("Failed to send pose data"); break; }
                    total_sent += sent;
                }
                if (total_sent != pose_data_size) { perror("Failed to send complete pose data"); break; }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    });

    // ROI visualization and multi-object tracking using template matching.
    std::thread roi_visualization_thread([&]() {
        while (is_not_end) {
            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (!shared_data.color_frame.empty()) {
                    shared_data.color_frame.copyTo(frame);
                }
            }
            if (!frame.empty()) {
                cv::Mat displayFrame = frame.clone();
                // Draw the currently drawn box (if any).
                {
                    std::lock_guard<std::mutex> lock(box_mutex);
                    if (drawing && currentBox.area() > 0) {
                        cv::rectangle(displayFrame, currentBox, cv::Scalar(0, 255, 0), 2);
                    }
                }
                // Process each tracked object.
                {
                    std::lock_guard<std::mutex> lock(objects_mutex);
                    cv::Mat gray;
                    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                    for (auto &obj : trackedObjects) {
                        // Ensure the object's box is within the image.
                        cv::Rect validBox = obj.objectBox & cv::Rect(0, 0, gray.cols, gray.rows);
                        // If the template is not yet set, initialize it.
                        if (obj.objectTemplate.empty() && validBox.area() > 0) {
                            obj.objectTemplate = gray(validBox).clone();
                        } else if (!obj.objectTemplate.empty()) {
                            // Run template matching on the current frame.
                            cv::Mat result;
                            int result_cols = gray.cols - obj.objectTemplate.cols + 1;
                            int result_rows = gray.rows - obj.objectTemplate.rows + 1;
                            result.create(result_rows, result_cols, CV_32FC1);
                            cv::matchTemplate(gray, obj.objectTemplate, result, cv::TM_CCOEFF_NORMED);
                            double minVal, maxVal;
                            cv::Point minLoc, maxLoc;
                            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
                            // Update the object's bounding box based on the best match.
                            obj.objectBox = cv::Rect(maxLoc.x, maxLoc.y, obj.objectTemplate.cols, obj.objectTemplate.rows);
                            // Optionally update the template for adaptation:
                            // obj.objectTemplate = gray(obj.objectBox & cv::Rect(0, 0, gray.cols, gray.rows)).clone();
                        }
                        // Draw the object's bounding box.
                        cv::rectangle(displayFrame, obj.objectBox, cv::Scalar(255, 0, 0), 2);
                    }
                }
                cv::imshow("ROI Selector", displayFrame);
            }
            if (cv::waitKey(1) == 27) { // Exit if ESC is pressed.
                is_not_end = false;
                break;
            }
        }
    });

    // Run viewer if requested.
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

    roi_visualization_thread.join();
    data_sender_thread.join();
    slam_thread.join();

    close(sock);
    cap.release();

    std::sort(track_times.begin(), track_times.end());
    double total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "Median tracking time: " << track_times.at(track_times.size() / 2) << " [s]" << std::endl;
    std::cout << "Mean tracking time: " << total_track_time / track_times.size() << " [s]" << std::endl;

    if (!map_db_path.empty()) {
        if (!slam->save_map_database(map_db_path)) {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------
// Main function.
int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    backward::SignalHandling sh;
#endif

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
        std::cerr << std::endl << op << std::endl;
        return EXIT_FAILURE;
    }
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
        std::cerr << "invalid arguments" << std::endl << std::endl << op << std::endl;
        return EXIT_FAILURE;
    }

    std::string viewer_string;
    if (viewer->is_set()) {
        viewer_string = viewer->value();
        if (viewer_string != "pangolin_viewer" &&
            viewer_string != "socket_publisher" &&
            viewer_string != "iridescence_viewer" &&
            viewer_string != "none") {
            std::cerr << "invalid arguments (--viewer)" << std::endl << std::endl << op << std::endl;
            return EXIT_FAILURE;
        }
#ifndef HAVE_PANGOLIN_VIEWER
        if (viewer_string == "pangolin_viewer") {
            std::cerr << "pangolin_viewer not linked" << std::endl << std::endl << op << std::endl;
            return EXIT_FAILURE;
        }
#endif
#ifndef HAVE_IRIDESCENCE_VIEWER
        if (viewer_string == "iridescence_viewer") {
            std::cerr << "iridescence_viewer not linked" << std::endl << std::endl << op << std::endl;
            return EXIT_FAILURE;
        }
#endif
#ifndef HAVE_SOCKET_PUBLISHER
        if (viewer_string == "socket_publisher") {
            std::cerr << "socket_publisher not linked" << std::endl << std::endl << op << std::endl;
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

    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    spdlog::set_level(spdlog::level::from_str(log_level->value()));

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
        } else {
            if (!slam->load_map_database(path)) {
                return EXIT_FAILURE;
            }
        }
    }
    slam->startup(need_initialize);
    if (disable_mapping->is_set()) {
        slam->disable_mapping_module();
    } else if (temporal_mapping->is_set()) {
        slam->enable_temporal_mapping();
        slam->disable_loop_detector();
    }

    int ret = 0;
    std::string camera_name = cfg->yaml_node_["Camera"]["name"].as<std::string>();
    if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        if (camera_name == "ZED2") {
            ret = mono_tracking_phone(slam,
                                      cfg,
                                      cam_num->value(),
                                      mask_img_path->value(),
                                      scale->value(),
                                      map_db_path_out->value(),
                                      viewer_string);
        } else if (camera_name == "Intel RealSense D435") {
            // Support for RealSense can be added here.
        }
    } else if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Stereo) {
        return 0;
    } else {
        throw std::runtime_error("Invalid setup type: " + slam->get_camera()->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return ret;
}

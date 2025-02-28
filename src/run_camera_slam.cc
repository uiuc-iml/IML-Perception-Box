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

#include <ghc/filesystem.hpp>
#include <arpa/inet.h> // For socket functions
#include <sys/socket.h> 
#include <unistd.h> 

namespace fs = ghc::filesystem;

#ifdef USE_STACK_TRACE_LOGGER
#include <backward.hpp>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif


int mono_tracking(const std::shared_ptr<stella_vslam::system>& slam,
                  const std::shared_ptr<stella_vslam::config>& cfg,
                  const unsigned int cam_num,
                  const std::string& mask_img_path,
                  const float scale,
                  const std::string& map_db_path,
                  const std::string& viewer_string) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    std::cout << cv::getBuildInformation() << std::endl;
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        slam->shutdown();
        return EXIT_FAILURE;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(5000);  // Set a port
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Replace with server IP
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }
    


    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> viewer;
    if (viewer_string == "pangolin_viewer") {
        viewer = std::make_shared<pangolin_viewer::viewer>(
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
        iridescence_viewer->add_checkbox("Pause", [&is_paused, &mtx_pause](bool check) {
            std::lock_guard<std::mutex> lock(mtx_pause);
            is_paused = check;
        });
        iridescence_viewer->add_button("Step", [&step_count, &mtx_step] {
            std::lock_guard<std::mutex> lock(mtx_step);
            step_count++;
        });
        iridescence_viewer->add_button("Reset", [&is_paused, &mtx_pause, &slam] {
            slam->request_reset();
        });
        iridescence_viewer->add_button("Save and exit", [&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate, &slam, &iridescence_viewer] {
            std::lock_guard<std::mutex> lock1(mtx_pause);
            is_paused = false;
            std::lock_guard<std::mutex> lock2(mtx_terminate);
            terminate_is_requested = true;
            iridescence_viewer->request_terminate();
        });
        iridescence_viewer->add_close_callback([&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate] {
            std::lock_guard<std::mutex> lock1(mtx_pause);
            is_paused = false;
            std::lock_guard<std::mutex> lock2(mtx_terminate);
            terminate_is_requested = true;
        });
    }
#endif
#ifdef HAVE_SOCKET_PUBLISHER
    std::shared_ptr<socket_publisher::publisher> publisher;
    if (viewer_string == "socket_publisher") {
        publisher = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"),
            slam,
            slam->get_frame_publisher(),
            slam->get_map_publisher());
    }
#endif

    auto video = cv::VideoCapture(cam_num);
    if (!video.isOpened()) {
        spdlog::critical("cannot open a camera {}", cam_num);
        slam->shutdown();
        return EXIT_FAILURE;
    }

    // Get frame properties
    int frame_width = 2208;
    int frame_height = 1104;
    int fps = 15;
    if (fps == 0) fps = 30; // Default to 30 FPS if unable to get FPS

    // Define GStreamer pipeline for streaming over UDP
    // std::string gst_pipeline = "appsrc ! videoconvert ! video/x-raw,format=I420,width=2208,height=1242 ! "
    //                        "shmsink socket-path=/tmp/mono_shm sync=false wait-for-connection=true";


    // cv::VideoWriter video_writer(gst_pipeline, cv::CAP_GSTREAMER, 0, fps, cv::Size(frame_width, frame_height), true);
    // if (!video_writer.isOpened()) {
    //     spdlog::critical("Failed to open video writer with GStreamer pipeline.");
    //     slam->shutdown();
    //     close(sock);
    //     return EXIT_FAILURE;
    // }

    cv::Mat frame;
    
    std::vector<double> track_times;

    unsigned int num_frame = 0;

    bool is_not_end = true;
    // run the slam in another thread
    std::thread thread([&]() {
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
            // check if the termination of slam system is requested or not
            {
                std::lock_guard<std::mutex> lock(mtx_terminate);
                if (terminate_is_requested) {
                    break;
                }
            }
#else
            // check if the termination of slam system is requested or not
            if (slam->terminate_is_requested()) {
                break;
            }
#endif

            is_not_end = video.read(frame);
            if (frame.empty()) {
                continue;
            }
            std::cout << "Size before:" <<frame.size() << std::endl;
            frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
            std::cout << "Size after:" <<frame.size() << std::endl;
            if (scale != 1.0) {
                cv::resize(frame, frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
            }
            int img_size = frame.total() * frame.elemSize();
            int bytes_sent = send(sock, frame.data, img_size, 0);
            if (bytes_sent != img_size) {
                perror("Failed to send frame");
                break;
            }
            // video_writer.write(frame);
            const auto tp_1 = std::chrono::steady_clock::now();

            // input the current frame and estimate the camera pose
            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
            slam->feed_monocular_frame(frame, timestamp, mask);

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

            ++num_frame;
        }

        // wait until the loop BA is finished
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    });

    // run the viewer in the current thread
    if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer->run();
#endif
    }
    if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
        iridescence_viewer->run();
#endif
    }
    if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        publisher->run();
#endif
    }
    // video_writer.release();
    close(sock);

    thread.join();

    // shutdown the slam process
    slam->shutdown();

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;

    if (!map_db_path.empty()) {
        if (!slam->save_map_database(map_db_path)) {
            return EXIT_FAILURE;
        }
    }


    return EXIT_SUCCESS;
}


































int mono_tracking_zed(const std::shared_ptr<stella_vslam::system>& slam,
                      const std::shared_ptr<stella_vslam::config>& cfg,
                      const unsigned int cam_num,
                      const std::string& mask_img_path,
                      const float scale,
                      const std::string& map_db_path,
                      const std::string& viewer_string) {
    // Load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    std::cout << cv::getBuildInformation() << std::endl;

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        slam->shutdown();
        return EXIT_FAILURE;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(5000);  // Set a port
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Replace with server IP
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    // Create a viewer object and pass the frame_publisher and the map_publisher
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> viewer;
    if (viewer_string == "pangolin_viewer") {
        viewer = std::make_shared<pangolin_viewer::viewer>(
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
        iridescence_viewer->add_button("Save and exit", [&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate, &slam, &iridescence_viewer] {
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
    std::shared_ptr<socket_publisher::publisher> publisher;
    if (viewer_string == "socket_publisher") {
        publisher = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"),
            slam,
            slam->get_frame_publisher(),
            slam->get_map_publisher());
    }
#endif

    auto video = cv::VideoCapture(cam_num);
    if (!video.isOpened()) {
        spdlog::critical("cannot open a camera {}", cam_num);
        slam->shutdown();
        return EXIT_FAILURE;
    }
    auto camera_node = stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "Camera");
    int fps = camera_node["fps"].as<int>(30); // Default to 30 if not specified
    int width = camera_node["cols"].as<int>(640); // Default width
    int height = camera_node["rows"].as<int>(480); // Default height

    video.set(cv::CAP_PROP_FRAME_WIDTH, width*2);
    video.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    video.set(cv::CAP_PROP_FPS, fps);

    cv::Mat frame;
    std::vector<double> track_times;
    unsigned int num_frame = 0;
    bool is_not_end = true;
    std::mutex frame_mutex;

    // Thread for video frame sending
    std::thread video_thread([&]() {
        while (is_not_end) {
            cv::Mat local_frame;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (!frame.empty()) {
                    cv::Mat frame2 = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
                    frame2.copyTo(local_frame);
                    std::cout << "Size before:" << frame2.size() << std::endl;
                }
            }
            if (!local_frame.empty()) {
                int img_size = local_frame.total() * local_frame.elemSize();
                int bytes_sent = send(sock, local_frame.data, img_size, 0);
                if (bytes_sent != img_size) {
                    perror("Failed to send frame");
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // Adjust for frame rate (about 30 fps)
        }
    });

    // Main SLAM processing thread
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
            // Check if the termination of SLAM system is requested
            {
                std::lock_guard<std::mutex> lock(mtx_terminate);
                if (terminate_is_requested) {
                    break;
                }
            }
#else
            if (slam->terminate_is_requested()) {
                break;
            }
#endif

            is_not_end = video.read(frame);
            if (frame.empty()) {
                continue;
            }
            cv::Mat frame_vslam;

            std::cout << "Size before:" << frame.size() << std::endl;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                frame_vslam = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
                if (scale != 1.0) {
                    cv::resize(frame_vslam , frame_vslam , cv::Size(), scale, scale, cv::INTER_LINEAR);
                }
            }
            std::cout << "Size after:" << frame.size() << std::endl;
            const auto tp_1 = std::chrono::steady_clock::now();

            // Input the current frame and estimate the camera pose
            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
            slam->feed_monocular_frame(frame_vslam , timestamp, mask);
            auto tracking_state = slam->get_frame_publisher()->get_tracking_state();

            // Check if the SLAM system is currently tracking
            if (tracking_state == "Tracking") {
                // Get the current camera pose from the map publisher
                Eigen::Matrix4d cam_pose = slam->get_map_publisher()->get_current_cam_pose();

                // Print the pose matrix
                std::cout << "Frame " << num_frame << " Pose:\n" << cam_pose << std::endl;

                // Extract rotation and translation
                Eigen::Matrix3d rotation = cam_pose.block<3,3>(0,0);
                Eigen::Vector3d translation = cam_pose.block<3,1>(0,3);

                // Convert rotation matrix to Euler angles (yaw, pitch, roll)
                Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0);
                Eigen::Vector3d euler_angles_deg = euler_angles * 180.0 / M_PI;

                // Print translation and rotation
                std::cout << "Translation (x, y, z): " << translation.transpose() << std::endl;
                std::cout << "Rotation (yaw, pitch, roll) in degrees: " << euler_angles_deg.transpose() << std::endl;
            } else {
                std::cout << "Frame " << num_frame << " tracking state: " << tracking_state << std::endl;
            }

            const auto tp_2 = std::chrono::steady_clock::now();
            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

            ++num_frame;
        }

        // Wait until the loop BA is finished
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    });

    // Run the viewer in the current thread
    if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer->run();
#endif
    }
    if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
        iridescence_viewer->run();
#endif
    }
    if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        publisher->run();
#endif
    }

    close(sock);
    video_thread.join();
    slam_thread.join();

    // Shutdown the SLAM process
    slam->shutdown();
    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;

    if (!map_db_path.empty()) {
        if (!slam->save_map_database(map_db_path)) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}











int mono_tracking_zed_pose(const std::shared_ptr<stella_vslam::system>& slam,
                      const std::shared_ptr<stella_vslam::config>& cfg,
                      const unsigned int cam_num,
                      const std::string& mask_img_path,
                      const float scale,
                      const std::string& map_db_path,
                      const std::string& viewer_string) {
    // Load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    std::cout << cv::getBuildInformation() << std::endl;

    // Socket initialization code
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        slam->shutdown();
        return EXIT_FAILURE;
    }
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;

    const char* ip_address = cfg->yaml_node_["SocketPublisher"]["address"].as<std::string>().c_str();
    int port_id = cfg->yaml_node_["SocketPublisher"]["port"].as<int>();
    server_addr.sin_port = htons(port_id);  // Set a port
    server_addr.sin_addr.s_addr = inet_addr(ip_address); // Replace with server IP
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    // Create a viewer object and pass the frame_publisher and the map_publisher
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> viewer;
    if (viewer_string == "pangolin_viewer") {
        viewer = std::make_shared<pangolin_viewer::viewer>(
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
        iridescence_viewer->add_button("Save and exit", [&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate, &slam, &iridescence_viewer] {
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
    std::shared_ptr<socket_publisher::publisher> publisher;
    if (viewer_string == "socket_publisher") {
        publisher = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"),
            slam,
            slam->get_frame_publisher(),
            slam->get_map_publisher());
    }
#endif

    auto video = cv::VideoCapture(cam_num);
    if (!video.isOpened()) {
        spdlog::critical("cannot open a camera {}", cam_num);
        slam->shutdown();
        return EXIT_FAILURE;
    }
    auto camera_node = stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "Camera");
    int fps = camera_node["fps"].as<int>(30); // Default to 30 if not specified
    int width = camera_node["cols"].as<int>(640); // Default width
    int height = camera_node["rows"].as<int>(480); // Default height

    video.set(cv::CAP_PROP_FRAME_WIDTH, width * 2);
    video.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    video.set(cv::CAP_PROP_FPS, fps);

    // Define shared data structure
    struct FramePoseData {
        cv::Mat frame;
        Eigen::Matrix4d pose;
        bool has_pose = false;
    };
    std::mutex data_mutex;
    FramePoseData shared_data;

    bool is_not_end = true;
    unsigned int num_frame = 0;
    std::vector<double> track_times;

    // Thread for SLAM processing
    std::thread slam_thread([&]() {
        cv::Mat frame;
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
            // Check if the termination of SLAM system is requested
            {
                std::lock_guard<std::mutex> lock(mtx_terminate);
                if (terminate_is_requested) {
                    break;
                }
            }
#else
            if (slam->terminate_is_requested()) {
                break;
            }
#endif

            // Read frame
            is_not_end = video.read(frame);
            if (frame.empty()) {
                continue;
            }

            cv::Mat frame_vslam = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
            if (scale != 1.0) {
                cv::resize(frame_vslam, frame_vslam, cv::Size(), scale, scale, cv::INTER_LINEAR);
            }

            const auto tp_1 = std::chrono::steady_clock::now();

            // Input the current frame and estimate the camera pose
            auto now = std::chrono::system_clock::now();
            double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
            slam->feed_monocular_frame(frame_vslam, timestamp, mask);

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

            const auto tp_2 = std::chrono::steady_clock::now();
            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

            // Write frame and pose to shared data
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                frame_vslam.copyTo(shared_data.frame);
                if (has_pose) {
                    shared_data.pose = cam_pose;
                    shared_data.has_pose = true;
                } else {
                    shared_data.has_pose = false;
                }
            }

            ++num_frame;
        }

        // Wait until the loop BA is finished
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    });

    // Thread for sending image and pose over the socket
    std::thread data_sender_thread([&]() {
        while (is_not_end) {
            cv::Mat local_frame;
            Eigen::Matrix4d local_pose;
            bool local_has_pose = false;

            // Read shared data
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (!shared_data.frame.empty()) {
                    shared_data.frame.copyTo(local_frame);
                    local_has_pose = shared_data.has_pose;
                    if (local_has_pose) {
                        local_pose = shared_data.pose;
                    }
                }
            }

            if (!local_frame.empty()) {
                // Serialize image
                std::vector<uchar> buf;
                cv::imencode(".jpg", local_frame, buf);
                int img_size = buf.size();
                int32_t net_img_size = htonl(img_size);

                // Serialize pose
                double pose_array[16];
                if (local_has_pose) {
                    memcpy(pose_array, local_pose.data(), sizeof(pose_array));
                } else {
                    std::fill(std::begin(pose_array), std::end(pose_array), 0.0);
                }

                // Send image size
                int sent = send(sock, &net_img_size, sizeof(net_img_size), 0);
                if (sent != sizeof(net_img_size)) {
                    perror("Failed to send image size");
                    break;
                }

                // Send image data
                int total_sent = 0;
                while (total_sent < img_size) {
                    sent = send(sock, buf.data() + total_sent, img_size - total_sent, 0);
                    if (sent <= 0) {
                        perror("Failed to send image data");
                        break;
                    }
                    total_sent += sent;
                }
                if (total_sent != img_size) {
                    perror("Failed to send complete image data");
                    break;
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

            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // Adjust for frame rate (about 30 fps)
        }
    });

    // Run the viewer in the current thread
    if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer->run();
#endif
    }
    if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
        iridescence_viewer->run();
#endif
    }
    if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        publisher->run();
#endif
    }

    // Close the socket
    close(sock);

    // Wait for threads to finish
    data_sender_thread.join();
    slam_thread.join();

    // Shutdown the SLAM process
    slam->shutdown();

    // Output tracking times
    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;

    if (!map_db_path.empty()) {
        if (!slam->save_map_database(map_db_path)) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}

















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
int mono_tracking_zed_pose_depth(const std::shared_ptr<stella_vslam::system>& slam,
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

    // Set ZED camera initialization parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720; // You can set this from cfg if needed
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init_params.coordinate_units = sl::UNIT::MILLIMETER;   // Depth in millimeters

    // Open the ZED camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error opening ZED camera: " << sl::toString(err) << std::endl;
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    // Define a shared data structure to hold frames and pose data
    struct FramePoseData {
        cv::Mat color_frame;
        cv::Mat depth_frame;
        Eigen::Matrix4d pose;
        bool has_pose = false;
    };
    std::mutex data_mutex;
    FramePoseData shared_data;

    // Variables for controlling the loop and tracking times
    bool is_not_end = true;
    unsigned int num_frame = 0;
    std::vector<double> track_times;

    // Thread for SLAM processing
    std::thread slam_thread([&]() {
        while (is_not_end) {
            // Pause functionality for the viewer (if using iridescence_viewer)
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

            // Check for termination request
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

            // Capture a new frame from the ZED camera
            if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
                // Retrieve the RGB image and depth map
                sl::Mat zed_image;
                sl::Mat zed_depth;
                zed.retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
                zed.retrieveMeasure(zed_depth, sl::MEASURE::DEPTH, sl::MEM::CPU);

                // Convert ZED image to OpenCV format (color frame)
                cv::Mat rgb_image = cv::Mat(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
                cv::Mat color_frame;
                cv::cvtColor(rgb_image, color_frame, cv::COLOR_BGRA2BGR);

                // Convert ZED depth map to OpenCV format (depth frame)
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

// Stella-VSLAM (headers depend on your setup)
#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>
#include <stella_vslam/util/yaml.h>

// If you have these viewers, include them:
#ifdef HAVE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
#include <iridescence_viewer/viewer.h>
#endif
#ifdef HAVE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

// ------------------------------------
// EKF code (basic) for IMU
// ------------------------------------
struct State {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation; // track orientation as a quaternion
};

class InertialEKF {
public:
    InertialEKF() {
        // state vector: [px, py, pz, vx, vy, vz]
        x_ = Eigen::VectorXd::Zero(6);
        // covariance
        P_ = Eigen::MatrixXd::Identity(6, 6) * 1e-3;
        // process noise
        Q_ = Eigen::MatrixXd::Identity(6, 6) * 1e-4;
        // measurement noise for velocity (ZUPT, 3x3)
        R_ = Eigen::MatrixXd::Identity(3, 3) * 1e-2;
    }

    // Predict step
    void predict(const Eigen::Vector3d& acc_world, double dt) {
        // x_(0..2) = position, x_(3..5) = velocity
        Eigen::VectorXd x_pred = x_;
        x_pred.segment<3>(0) += x_.segment<3>(3) * dt + 0.5 * acc_world * dt * dt;
        x_pred.segment<3>(3) += acc_world * dt;

        // Jacobian F
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6,6);
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;

        P_ = F * P_ * F.transpose() + Q_;
        x_ = x_pred;
    }

    // ZUPT update (if stationary)
    void updateZUPT(const Eigen::Vector3d& meas_vel) {
        // H = [0_3x3 | I_3x3]
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
        H.block<3,3>(0,3) = Eigen::Matrix3d::Identity();

        Eigen::Vector3d v_pred = x_.segment<3>(3);
        Eigen::Vector3d y = meas_vel - v_pred; // measurement - prediction

        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        x_ = x_ + K * y;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
        P_ = (I - K * H) * P_;
    }

    Eigen::Vector3d getPosition() const {
        return x_.segment<3>(0);
    }
    Eigen::Vector3d getVelocity() const {
        return x_.segment<3>(3);
    }
    void setState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) {
        x_.segment<3>(0) = pos;
        x_.segment<3>(3) = vel;
    }

private:
    // [px, py, pz, vx, vy, vz]
    Eigen::VectorXd x_;
    // Covariance
    Eigen::MatrixXd P_;
    // Process noise
    Eigen::MatrixXd Q_;
    // Measurement noise
    Eigen::MatrixXd R_;
};

// Check if stationary: naive approach
bool isStationary(const Eigen::Vector3d& gyro_deg_s,
                  const Eigen::Vector3d& accel_body,
                  double gyro_threshold_deg_s = 1.0,
                  double accel_threshold_m_s2 = 0.1) {
    // If angular velocity is near zero, and accel magnitude ~ 9.81
    if ((std::fabs(gyro_deg_s.x()) < gyro_threshold_deg_s) &&
        (std::fabs(gyro_deg_s.y()) < gyro_threshold_deg_s) &&
        (std::fabs(gyro_deg_s.z()) < gyro_threshold_deg_s))
    {
        double accel_mag = accel_body.norm();
        double g = 9.81;
        if (std::fabs(accel_mag - g) < accel_threshold_m_s2) {
            return true;
        }
    }
    return false;
}

// ------------------------------------
// The combined function
// ------------------------------------
int mono_tracking_zed_pose_depth_with_imu(
    const std::shared_ptr<stella_vslam::system>& slam,
    const std::shared_ptr<stella_vslam::config>& cfg,
    const unsigned int cam_num,
    const std::string& mask_img_path,
    const float scale,
    const std::string& map_db_path,
    const std::string& viewer_string)
{
    // ------------------
    // Load mask if provided
    // ------------------
    cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // ------------------
    // Socket initialization
    // ------------------
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        slam->shutdown();
        return EXIT_FAILURE;
    }
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    std::string ip_address = cfg->yaml_node_["SocketPublisher"]["address"].as<std::string>();
    int port_id = cfg->yaml_node_["SocketPublisher"]["port"].as<int>();
    server_addr.sin_port = htons(port_id);
    server_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    // ------------------
    // Viewer initialization
    // ------------------
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> pangolin_viewer;
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
    std::shared_ptr<iridescence_viewer::viewer> iridescence_viewer;
    std::mutex mtx_pause;
    bool is_paused = false;
    std::mutex mtx_terminate;
    bool terminate_is_requested = false;
    std::mutex mtx_step;
    unsigned int step_count = 0;
#endif
#ifdef HAVE_SOCKET_PUBLISHER
    std::shared_ptr<socket_publisher::publisher> socket_publisher;
#endif

    if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        pangolin_viewer = std::make_shared<pangolin_viewer::viewer>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"),
            slam,
            slam->get_frame_publisher(),
            slam->get_map_publisher());
#endif
    }
    else if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
        iridescence_viewer = std::make_shared<iridescence_viewer::viewer>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "IridescenceViewer"),
            slam->get_frame_publisher(),
            slam->get_map_publisher());
        // Add controls
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
        iridescence_viewer->add_button("Save and exit", 
            [&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate, &iridescence_viewer] {
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
        iridescence_viewer->add_close_callback(
            [&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate] {
            {
                std::lock_guard<std::mutex> lock1(mtx_pause);
                is_paused = false;
            }
            {
                std::lock_guard<std::mutex> lock2(mtx_terminate);
                terminate_is_requested = true;
            }
        });
#endif
    }
    else if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        socket_publisher = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"),
            slam,
            slam->get_frame_publisher(),
            slam->get_map_publisher());
#endif
    }

    // ------------------
    // ZED camera initialization
    // ------------------
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init_params.coordinate_units = sl::UNIT::MILLIMETER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // For consistent IMU gravity direction

    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error opening ZED camera: " << sl::toString(err) << std::endl;
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    if (!zed.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::GYROSCOPE)) {
        std::cerr << "IMU data is not available on this camera model." << std::endl;
        slam->shutdown();
        close(sock);
        zed.close();
        return EXIT_FAILURE;
    }

    // Shared data structure for frames & pose
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

    // ------------------
    // IMU / EKF Setup
    // ------------------
    InertialEKF ekf;
    ekf.setState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()); // start at origin, zero velocity

    State state;
    state.position = Eigen::Vector3d::Zero();
    state.velocity = Eigen::Vector3d::Zero();
    state.orientation = Eigen::Quaterniond::Identity();

    auto previous_time = std::chrono::steady_clock::now();
    // Gravity in Y-down if Y is up => gravity = (0, -9.81, 0)
    Eigen::Vector3d gravity(0, -9.81, 0);

    // ------------------
    // SLAM thread
    // ------------------
    std::thread slam_thread([&]() {
        while (is_not_end) {
#ifdef HAVE_IRIDESCENCE_VIEWER
            // Pause logic if using iridescence_viewer
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

            // Termination check from the viewer
            {
                std::lock_guard<std::mutex> lock(mtx_terminate);
                if (terminate_is_requested) {
                    is_not_end = false;
                    break;
                }
            }
#else
            // If not using iridescence_viewer, check SLAM termination
            if (slam->terminate_is_requested()) {
                is_not_end = false;
                break;
            }
#endif

            // Grab from ZED
            if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
                sl::Mat zed_image, zed_depth;
                zed.retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
                zed.retrieveMeasure(zed_depth, sl::MEASURE::DEPTH, sl::MEM::CPU);

                // Convert to OpenCV
                cv::Mat rgb_image(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
                cv::Mat color_frame;
                cv::cvtColor(rgb_image, color_frame, cv::COLOR_BGRA2BGR);

                cv::Mat depth_frame(zed_depth.getHeight(), zed_depth.getWidth(), CV_32FC1, zed_depth.getPtr<sl::float1>(sl::MEM::CPU));

                // Scale if needed
                if (scale != 1.0f) {
                    cv::resize(color_frame, color_frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
                    cv::resize(depth_frame, depth_frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
                }

                // ----------------------
                // IMU Data
                // ----------------------
                sl::SensorsData sensors_data;
                zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT);

                auto angular_velocity = sensors_data.imu.angular_velocity;      // [deg/s]
                auto linear_acceleration = sensors_data.imu.linear_acceleration; // [m/s^2]
                auto zed_orientation = sensors_data.imu.pose.getOrientation();   // (ox, oy, oz, ow)

                // Convert orientation to Eigen
                Eigen::Quaterniond q_imu(
                    static_cast<double>(zed_orientation.ow),
                    static_cast<double>(zed_orientation.ox),
                    static_cast<double>(zed_orientation.oy),
                    static_cast<double>(zed_orientation.oz));
                q_imu.normalize();

                // Compute dt
                auto current_time = std::chrono::steady_clock::now();
                double dt = std::chrono::duration<double>(current_time - previous_time).count();
                previous_time = current_time;

                // Convert body accel to world frame
                Eigen::Vector3d acc_body(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
                Eigen::Vector3d acc_world = q_imu * acc_body;
                // Subtract gravity
                acc_world -= gravity;

                // EKF predict
                ekf.predict(acc_world, dt);

                // Possibly do ZUPT if stationary
                Eigen::Vector3d gyro_deg_s(angular_velocity.x, angular_velocity.y, angular_velocity.z);
                if (isStationary(gyro_deg_s, acc_body)) {
                    ekf.updateZUPT(Eigen::Vector3d::Zero());
                }

                // Update state
                state.position = ekf.getPosition();
                state.velocity = ekf.getVelocity();
                // For orientation, we take the IMU's orientation
                state.orientation = q_imu;

                // ----------------------
                // SLAM processing
                // ----------------------
                auto now_chrono = std::chrono::system_clock::now();
                double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(now_chrono.time_since_epoch()).count();

                auto tp_1 = std::chrono::steady_clock::now();
                slam->feed_monocular_frame(color_frame, timestamp, mask);

                auto tracking_state = slam->get_frame_publisher()->get_tracking_state();
                Eigen::Matrix4d cam_pose = Eigen::Matrix4d::Identity();
                bool has_pose = false;

                if (tracking_state == "Tracking") {
                    cam_pose = slam->get_map_publisher()->get_current_cam_pose();
                    has_pose = true;

                    // Print pose
                    std::cout << "Frame " << num_frame << " SLAM Pose:\n" << cam_pose << std::endl;
                }
                else {
                    std::cout << "Frame " << num_frame << " tracking state: " << tracking_state << std::endl;
                }

                auto tp_2 = std::chrono::steady_clock::now();
                double track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
                track_times.push_back(track_time);

                // ----------------------
                // Fuse SLAM pose & EKF
                // (Naive approach)
                // ----------------------
                // Use IMU orientation
                Eigen::Matrix3d R_imu = state.orientation.toRotationMatrix();

                // Weighted translation between SLAM & EKF
                double alpha = 0.7; // weighting for SLAM
                Eigen::Vector3d t_slam = cam_pose.block<3,1>(0,3);
                Eigen::Vector3d t_ekf = state.position;
                Eigen::Vector3d t_fused = alpha * t_slam + (1.0 - alpha) * t_ekf;

                Eigen::Matrix4d fused_pose = Eigen::Matrix4d::Identity();
                fused_pose.block<3,3>(0,0) = R_imu;
                fused_pose.block<3,1>(0,3) = t_fused;

                // Store in shared data
                {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    color_frame.copyTo(shared_data.color_frame);
                    depth_frame.copyTo(shared_data.depth_frame);
                    shared_data.has_pose = true;
                    shared_data.pose = fused_pose;
                }

                ++num_frame;
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        // Wait for loop closure / BA
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
        slam->shutdown();
    });

    // ------------------
    // Data sender thread
    // ------------------
    std::thread data_sender_thread([&]() {
        while (is_not_end) {
            cv::Mat color_local, depth_local;
            Eigen::Matrix4d local_pose;
            bool local_has_pose = false;

            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (!shared_data.color_frame.empty()) {
                    color_local = shared_data.color_frame.clone();
                    depth_local = shared_data.depth_frame.clone();
                    local_has_pose = shared_data.has_pose;
                    local_pose = shared_data.pose;
                }
            }

            // Send images + pose
            if (!color_local.empty()) {
                // 1) color image (JPEG)
                std::vector<uchar> buf;
                cv::imencode(".jpg", color_local, buf);
                int img_size = static_cast<int>(buf.size());
                int32_t net_img_size = htonl(img_size);
                int sent = send(sock, &net_img_size, sizeof(net_img_size), 0);
                if (sent != sizeof(net_img_size)) {
                    perror("Failed to send color image size");
                    break;
                }
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

                // 2) depth image (PNG)
                if (!depth_local.empty()) {
                    depth_local.convertTo(depth_local, CV_16U);
                    std::vector<uchar> depth_buf;
                    cv::imencode(".png", depth_local, depth_buf);
                    int depth_img_size = static_cast<int>(depth_buf.size());
                    int32_t net_depth_img_size = htonl(depth_img_size);

                    sent = send(sock, &net_depth_img_size, sizeof(net_depth_img_size), 0);
                    if (sent != sizeof(net_depth_img_size)) {
                        perror("Failed to send depth image size");
                        break;
                    }

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
                }
                else {
                    int32_t zero_size = 0;
                    int32_t net_depth_img_size = htonl(zero_size);
                    send(sock, &net_depth_img_size, sizeof(net_depth_img_size), 0);
                }

                // 3) pose data (16 doubles)
                double pose_array[16];
                if (local_has_pose) {
                    memcpy(pose_array, local_pose.data(), sizeof(pose_array));
                }
                else {
                    std::fill(std::begin(pose_array), std::end(pose_array), 0.0);
                }

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

            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
        }
    });

    // ------------------
    // Run viewer in the main thread
    // ------------------
    if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        pangolin_viewer->run();
#endif
    }
    else if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
        iridescence_viewer->run();
#endif
    }
    else if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        socket_publisher->run();
#endif
    }

    // Wait for threads
    data_sender_thread.join();
    slam_thread.join();

    // Cleanup
    close(sock);
    zed.close();

    // Print tracking times
    std::sort(track_times.begin(), track_times.end());
    double total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    if (!track_times.empty()) {
        std::cout << "Median tracking time: " 
                  << track_times.at(track_times.size() / 2) << " [s]" << std::endl;
        std::cout << "Mean tracking time: "
                  << (total_track_time / track_times.size()) << " [s]" << std::endl;
    }

    // Save map if requested
    if (!map_db_path.empty()) {
        if (!slam->save_map_database(map_db_path)) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}




















int mono_tracking_realsense(const std::shared_ptr<stella_vslam::system>& slam,
                      const std::shared_ptr<stella_vslam::config>& cfg,
                      const unsigned int cam_num,
                      const std::string& mask_img_path,
                      const float scale,
                      const std::string& map_db_path,
                      const std::string& viewer_string) {
    // Load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    std::cout << cv::getBuildInformation() << std::endl;

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        slam->shutdown();
        return EXIT_FAILURE;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(5000);  // Set a port
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Replace with server IP
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    // Create a viewer object and pass the frame_publisher and the map_publisher
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> viewer;
    if (viewer_string == "pangolin_viewer") {
        viewer = std::make_shared<pangolin_viewer::viewer>(
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
        iridescence_viewer->add_button("Save and exit", [&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate, &slam, &iridescence_viewer] {
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
    std::shared_ptr<socket_publisher::publisher> publisher;
    if (viewer_string == "socket_publisher") {
        publisher = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"),
            slam,
            slam->get_frame_publisher(),
            slam->get_map_publisher());
    }
#endif

    auto video = cv::VideoCapture(cam_num);
    if (!video.isOpened()) {
        spdlog::critical("cannot open a camera {}", cam_num);
        slam->shutdown();
        return EXIT_FAILURE;
    }
    auto camera_node = stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "Camera");
    int fps = camera_node["fps"].as<int>(30); // Default to 30 if not specified
    int width = camera_node["cols"].as<int>(640); // Default width
    int height = camera_node["rows"].as<int>(480); // Default height

    video.set(cv::CAP_PROP_FRAME_WIDTH, width);
    video.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    video.set(cv::CAP_PROP_FPS, fps);

    cv::Mat frame;
    std::vector<double> track_times;
    unsigned int num_frame = 0;
    bool is_not_end = true;
    std::mutex frame_mutex;

    // Thread for video frame sending
    std::thread video_thread([&]() {
        while (is_not_end) {
            cv::Mat local_frame;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (!frame.empty()) {
                    cv::Mat frame2 = frame;
                    frame2.copyTo(local_frame);
                }
            }
            if (!local_frame.empty()) {
                int img_size = local_frame.total() * local_frame.elemSize();
                int bytes_sent = send(sock, local_frame.data, img_size, 0);
                if (bytes_sent != img_size) {
                    perror("Failed to send frame");
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // Adjust for frame rate (about 30 fps)
        }
    });

    // Main SLAM processing thread
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
            // Check if the termination of SLAM system is requested
            {
                std::lock_guard<std::mutex> lock(mtx_terminate);
                if (terminate_is_requested) {
                    break;
                }
            }
#else
            if (slam->terminate_is_requested()) {
                break;
            }
#endif

            is_not_end = video.read(frame);
            if (frame.empty()) {
                continue;
            }
            cv::Mat frame_vslam;

            std::cout << "Size before:" << frame.size() << std::endl;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                frame_vslam = frame;
                if (scale != 1.0) {
                    cv::resize(frame_vslam , frame_vslam , cv::Size(), scale, scale, cv::INTER_LINEAR);
                }
            }
            std::cout << "Size after:" << frame.size() << std::endl;
            const auto tp_1 = std::chrono::steady_clock::now();

            // Input the current frame and estimate the camera pose
            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
            slam->feed_monocular_frame(frame_vslam , timestamp, mask);
            auto tracking_state = slam->get_frame_publisher()->get_tracking_state();

            if (tracking_state == "Tracking") {
                // Get the current camera pose from the map publisher
                Eigen::Matrix4d cam_pose = slam->get_map_publisher()->get_current_cam_pose();

                // Print the pose matrix
                std::cout << "Frame " << num_frame << " Pose:\n" << cam_pose << std::endl;

                // Extract rotation and translation
                Eigen::Matrix3d rotation = cam_pose.block<3,3>(0,0);
                Eigen::Vector3d translation = cam_pose.block<3,1>(0,3);

                // Convert rotation matrix to Euler angles (yaw, pitch, roll)
                Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0);
                Eigen::Vector3d euler_angles_deg = euler_angles * 180.0 / M_PI;

                // Print translation and rotation
                std::cout << "Translation (x, y, z): " << translation.transpose() << std::endl;
                std::cout << "Rotation (yaw, pitch, roll) in degrees: " << euler_angles_deg.transpose() << std::endl;
            } else {
                std::cout << "Frame " << num_frame << " tracking state: " << tracking_state << std::endl;
            }

            const auto tp_2 = std::chrono::steady_clock::now();
            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

            ++num_frame;
        }

        // Wait until the loop BA is finished
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    });

    // Run the viewer in the current thread
    if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer->run();
#endif
    }
    if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
        iridescence_viewer->run();
#endif
    }
    if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        publisher->run();
#endif
    }

    close(sock);
    video_thread.join();
    slam_thread.join();

    // Shutdown the SLAM process
    slam->shutdown();
    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;

    if (!map_db_path.empty()) {
        if (!slam->save_map_database(map_db_path)) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}








int mono_tracking_realsense_pose(const std::shared_ptr<stella_vslam::system>& slam,
                            const std::shared_ptr<stella_vslam::config>& cfg,
                            const unsigned int cam_num,
                            const std::string& mask_img_path,
                            const float scale,
                            const std::string& map_db_path,
                            const std::string& viewer_string) {
    // Load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    std::cout << cv::getBuildInformation() << std::endl;

    // Socket initialization code
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        slam->shutdown();
        return EXIT_FAILURE;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    const char* ip_address = cfg->yaml_node_["SocketPublisher"]["address"].as<std::string>().c_str();
    int port_id = cfg->yaml_node_["SocketPublisher"]["port"].as<int>();
    server_addr.sin_port = htons(port_id);  // Set a port
    server_addr.sin_addr.s_addr = inet_addr(ip_address); // Replace with server IP
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    // Create a viewer object and pass the frame_publisher and the map_publisher
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> viewer;
    if (viewer_string == "pangolin_viewer") {
        viewer = std::make_shared<pangolin_viewer::viewer>(
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
        iridescence_viewer->add_button("Save and exit", [&is_paused, &mtx_pause, &terminate_is_requested, &mtx_terminate, &slam, &iridescence_viewer] {
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
    std::shared_ptr<socket_publisher::publisher> publisher;
    if (viewer_string == "socket_publisher") {
        publisher = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"),
            slam,
            slam->get_frame_publisher(),
            slam->get_map_publisher());
    }
#endif

    auto video = cv::VideoCapture(cam_num);
    if (!video.isOpened()) {
        spdlog::critical("cannot open a camera {}", cam_num);
        slam->shutdown();
        return EXIT_FAILURE;
    }
    auto camera_node = stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "Camera");
    int fps = camera_node["fps"].as<int>(30); // Default to 30 if not specified
    int width = camera_node["cols"].as<int>(640); // Default width
    int height = camera_node["rows"].as<int>(480); // Default height

    video.set(cv::CAP_PROP_FRAME_WIDTH, width);
    video.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    video.set(cv::CAP_PROP_FPS, fps);

    // Define shared data structure
    struct FramePoseData {
        cv::Mat frame;
        Eigen::Matrix4d pose;
        bool has_pose = false;
    };
    std::mutex data_mutex;
    FramePoseData shared_data;

    bool is_not_end = true;
    unsigned int num_frame = 0;
    std::vector<double> track_times;

    // Thread for SLAM processing
    std::thread slam_thread([&]() {
        cv::Mat frame;
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
            // Check if the termination of SLAM system is requested
            {
                std::lock_guard<std::mutex> lock(mtx_terminate);
                if (terminate_is_requested) {
                    break;
                }
            }
#else
            if (slam->terminate_is_requested()) {
                break;
            }
#endif

            // Read frame
            is_not_end = video.read(frame);
            if (frame.empty()) {
                continue;
            }

            cv::Mat frame_vslam = frame;
            if (scale != 1.0) {
                cv::resize(frame_vslam, frame_vslam, cv::Size(), scale, scale, cv::INTER_LINEAR);
            }

            const auto tp_1 = std::chrono::steady_clock::now();

            // Input the current frame and estimate the camera pose
            auto now = std::chrono::system_clock::now();
            double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
            slam->feed_monocular_frame(frame_vslam, timestamp, mask);

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

            const auto tp_2 = std::chrono::steady_clock::now();
            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

            // Write frame and pose to shared data
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                frame_vslam.copyTo(shared_data.frame);
                if (has_pose) {
                    shared_data.pose = cam_pose;
                    shared_data.has_pose = true;
                } else {
                    shared_data.has_pose = false;
                }
            }

            ++num_frame;
        }

        // Wait until the loop BA is finished
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    });

    // Thread for sending image and pose over the socket
    std::thread data_sender_thread([&]() {
        while (is_not_end) {
            cv::Mat local_frame;
            Eigen::Matrix4d local_pose;
            bool local_has_pose = false;

            // Read shared data
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (!shared_data.frame.empty()) {
                    shared_data.frame.copyTo(local_frame);
                    local_has_pose = shared_data.has_pose;
                    if (local_has_pose) {
                        local_pose = shared_data.pose;
                    }
                }
            }

            if (!local_frame.empty()) {
                // Serialize image
                std::vector<uchar> buf;
                cv::imencode(".jpg", local_frame, buf);
                int img_size = buf.size();
                int32_t net_img_size = htonl(img_size);

                // Serialize pose
                double pose_array[16];
                if (local_has_pose) {
                    memcpy(pose_array, local_pose.data(), sizeof(pose_array));
                } else {
                    std::fill(std::begin(pose_array), std::end(pose_array), 0.0);
                }

                // Send image size
                int sent = send(sock, &net_img_size, sizeof(net_img_size), 0);
                if (sent != sizeof(net_img_size)) {
                    perror("Failed to send image size");
                    break;
                }

                // Send image data
                int total_sent = 0;
                while (total_sent < img_size) {
                    sent = send(sock, buf.data() + total_sent, img_size - total_sent, 0);
                    if (sent <= 0) {
                        perror("Failed to send image data");
                        break;
                    }
                    total_sent += sent;
                }
                if (total_sent != img_size) {
                    perror("Failed to send complete image data");
                    break;
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

            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // Adjust for frame rate (about 30 fps)
        }
    });

    // Run the viewer in the current thread
    if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer->run();
#endif
    }
    if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
        iridescence_viewer->run();
#endif
    }
    if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        publisher->run();
#endif
    }

    // Close the socket
    close(sock);

    // Wait for threads to finish
    data_sender_thread.join();
    slam_thread.join();

    // Shutdown the SLAM process
    slam->shutdown();

    // Output tracking times
    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;

    if (!map_db_path.empty()) {
        if (!slam->save_map_database(map_db_path)) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}









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
#include <librealsense2/rs.hpp> // RealSense API
#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/yaml.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>
#include <pangolin/pangolin.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

int mono_tracking_realsense_pose_depth(const std::shared_ptr<stella_vslam::system>& slam,
                                 const std::shared_ptr<stella_vslam::config>& cfg,
                                 const unsigned int cam_num,
                                 const std::string& mask_img_path,
                                 const float scale,
                                 const std::string& map_db_path,
                                 const std::string& viewer_string) {
    // Load mask image if provided
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // Configure RealSense pipeline for depth and color streams
    rs2::pipeline rs_pipe;
    rs2::config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    rs_cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    try {
        rs_pipe.start(rs_cfg);
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        slam->shutdown();
        return EXIT_FAILURE;
    }

    // Initialize a TCP socket for data transmission
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        slam->shutdown();
        return EXIT_FAILURE;
    }

    struct sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    const char* ip_address = cfg->yaml_node_["SocketPublisher"]["address"].as<std::string>().c_str();
    int port = cfg->yaml_node_["SocketPublisher"]["port"].as<int>();
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(ip_address);

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        slam->shutdown();
        close(sock);
        return EXIT_FAILURE;
    }

    // Viewer initialization based on input configuration
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

    // Shared data structure for SLAM frames and pose
    struct FramePoseData {
        cv::Mat color_frame, depth_frame;
        Eigen::Matrix4d pose;
        bool has_pose = false;
    };
    std::mutex data_mutex;
    FramePoseData shared_data;

    bool is_running = true;
    unsigned int num_frames = 0;
    std::vector<double> tracking_times;

    // SLAM processing thread
    std::thread slam_thread([&]() {
        while (is_running) {
            rs2::frameset frames;
            try {
                frames = rs_pipe.wait_for_frames();
            } catch (const rs2::error&) {
                break;
            }

            // Retrieve color and depth frames
            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();

            // Convert to OpenCV format
            cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data());
            cv::Mat depth_mat(cv::Size(640, 480), CV_16U, (void*)depth_frame.get_data());

            if (scale != 1.0) {
                cv::resize(color_mat, color_mat, cv::Size(), scale, scale, cv::INTER_LINEAR);
            }

            const auto start_time = std::chrono::steady_clock::now();

            // Feed the current frame to SLAM
            double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count();
            slam->feed_monocular_frame(color_mat, timestamp, mask);

            auto tracking_state = slam->get_frame_publisher()->get_tracking_state();
            Eigen::Matrix4d cam_pose;
            bool has_pose = false;

            if (tracking_state == "Tracking") {
                // Get the current camera pose from the map publisher
                cam_pose = slam->get_map_publisher()->get_current_cam_pose();
                has_pose = true;

                // Print the pose matrix
                std::cout << "Frame " << num_frames << " Pose:\n" << cam_pose << std::endl;

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
                std::cout << "Frame " << num_frames << " tracking state: " << tracking_state << std::endl;
            }

            const auto end_time = std::chrono::steady_clock::now();
            tracking_times.push_back(std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count());

            // Update shared data
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                color_mat.copyTo(shared_data.color_frame);
                depth_mat.copyTo(shared_data.depth_frame);
                shared_data.pose = cam_pose;
                shared_data.has_pose = has_pose;
            }

            ++num_frames;
        }

        slam->shutdown();
    });

    // Data transmission thread
    std::thread data_thread([&]() {
        while (is_running) {
            cv::Mat color_local, depth_local;
            Eigen::Matrix4d pose_local;
            bool has_pose_local = false;

            // Retrieve shared data
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (!shared_data.color_frame.empty()) {
                    shared_data.color_frame.copyTo(color_local);
                }
                if (!shared_data.depth_frame.empty()) {
                    shared_data.depth_frame.copyTo(depth_local);
                }
                has_pose_local = shared_data.has_pose;
                pose_local = shared_data.pose;
            }

            if (!color_local.empty()) {
                // Transmit color frame as JPEG
                std::vector<uchar> buf;
                cv::imencode(".jpg", color_local, buf);
                int size = buf.size();
                int32_t net_size = htonl(size);

                send(sock, &net_size, sizeof(net_size), 0);
                send(sock, buf.data(), size, 0);

                // Transmit depth frame as PNG
                if (!depth_local.empty()) {
                    // Normalize depth image for visualization
                    cv::Mat depth_normalized;
                    double minVal, maxVal;
                    cv::minMaxLoc(depth_local, &minVal, &maxVal);
                    depth_local.convertTo(depth_normalized, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

                    // Encode depth image
                    std::vector<uchar> depth_buf;
                    cv::imencode(".png", depth_normalized, depth_buf);
                    int depth_size = depth_buf.size();
                    int32_t depth_net_size = htonl(depth_size);

                    send(sock, &depth_net_size, sizeof(depth_net_size), 0);
                    send(sock, depth_buf.data(), depth_size, 0);
                } else {
                    // Send zero size if depth frame is empty
                    int32_t depth_net_size = htonl(0);
                    send(sock, &depth_net_size, sizeof(depth_net_size), 0);
                }

                // Always transmit pose data
                double pose_data[16];
                if (has_pose_local) {
                    std::memcpy(pose_data, pose_local.data(), sizeof(pose_data));
                } else {
                    // Send identity matrix or zeros if no pose is available
                    std::fill(pose_data, pose_data + 16, 0.0);
                    //pose_data[0] = pose_data[5] = pose_data[10] = pose_data[15] = 1.0; // Identity matrix
                }
                send(sock, pose_data, sizeof(pose_data), 0);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    });

    if (viewer_string == "pangolin_viewer") {
        #ifdef HAVE_PANGOLIN_VIEWER
        pangolin_viewer->run();
        #endif
    }

    // Wait for threads to complete
    slam_thread.join();
    data_thread.join();

    close(sock);

    // Save the map database
    if (!map_db_path.empty() && !slam->save_map_database(map_db_path)) {
        return EXIT_FAILURE;
    }

    // Output tracking performance
    std::sort(tracking_times.begin(), tracking_times.end());
    double total_time = std::accumulate(tracking_times.begin(), tracking_times.end(), 0.0);
    std::cout << "Median tracking time: " << tracking_times.at(tracking_times.size() / 2) << " s\n";
    std::cout << "Mean tracking time: " << total_time / tracking_times.size() << " s\n";

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
            ret = mono_tracking_zed_pose_depth_with_imu(slam,
                                    cfg,
                                    cam_num->value(),
                                    mask_img_path->value(),
                                    scale->value(),
                                    map_db_path_out->value(),
                                    viewer_string);
        }
        else if (camera_name == "Intel RealSense D435") {

            ret = mono_tracking_realsense_pose_depth (slam,
                                          cfg,
                                          cam_num->value(),
                                          mask_img_path->value(),
                                          scale->value(),
                                          map_db_path_out->value(),
                                          viewer_string);
        }
        else {
            // Default to the original mono_tracking function for other cameras
            //Make changes later to be more modular
            ret = mono_tracking(slam,
                                cfg,
                                cam_num->value(),
                                mask_img_path->value(),
                                scale->value(),
                                map_db_path_out->value(),
                                viewer_string);
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

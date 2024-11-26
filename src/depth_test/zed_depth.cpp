#include <sl/Camera.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
    // Create a ZED camera object
    sl::Camera zed;

    // Set initialization parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720; // Set resolution
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;  // Set depth mode
    init_params.coordinate_units = sl::UNIT::MILLIMETER;   // Depth in millimeters

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error opening ZED camera: " << sl::toString(err) << std::endl;
        return EXIT_FAILURE;
    }

    // Create containers for RGB image and depth map
    sl::Mat zed_image;
    sl::Mat zed_depth;

    // Loop to capture frames
    while (true) {
        // Capture a new frame
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            // Retrieve the RGB image and depth map
            zed.retrieveImage(zed_image, sl::VIEW::LEFT);
            zed.retrieveMeasure(zed_depth, sl::MEASURE::DEPTH);

            // Convert ZED image to OpenCV format
            cv::Mat rgb_image = cv::Mat(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::Mat rgb_image_bgr;
            cv::cvtColor(rgb_image, rgb_image_bgr, cv::COLOR_BGRA2BGR);

            // Convert ZED depth map to OpenCV format
            cv::Mat depth_image = cv::Mat(zed_depth.getHeight(), zed_depth.getWidth(), CV_32FC1, zed_depth.getPtr<sl::float1>(sl::MEM::CPU));

            // Create RGB-D image (store R, G, B, and Depth)
            cv::Mat rgbd_image(rgb_image_bgr.rows, rgb_image_bgr.cols, CV_32FC4);
            for (int i = 0; i < rgb_image_bgr.rows; i++) {
                for (int j = 0; j < rgb_image_bgr.cols; j++) {
                    cv::Vec3b rgb = rgb_image_bgr.at<cv::Vec3b>(i, j);  // RGB values
                    float depth = depth_image.at<float>(i, j);          // Depth value
                    cv::Vec4f &rgbd = rgbd_image.at<cv::Vec4f>(i, j);
                    rgbd[0] = rgb[2]; // R
                    rgbd[1] = rgb[1]; // G
                    rgbd[2] = rgb[0]; // B
                    rgbd[3] = depth;  // Depth
                }
            }

            // Display the images
            cv::imshow("RGB Image", rgb_image_bgr);
            cv::imshow("Depth Map", depth_image / 10000.0f); // Normalize for visualization (depth in millimeters)

            // Save the RGB-D image as a binary file or process further
            std::cout << "Press 's' to save the RGB-D image or 'q' to quit.\n";
            char key = cv::waitKey(1);
            if (key == 's') {
                // Save RGB-D image
                std::ofstream rgbd_file("rgbd_image.bin", std::ios::binary);
                if (rgbd_file.is_open()) {
                    rgbd_file.write((char *)rgbd_image.data, rgbd_image.total() * rgbd_image.elemSize());
                    rgbd_file.close();
                    std::cout << "RGB-D image saved to rgbd_image.bin\n";
                }
            } else if (key == 'q') {
                break;
            }
        }
    }

    // Close the camera
    zed.close();

    return 0;
}

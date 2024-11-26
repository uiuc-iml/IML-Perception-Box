#include <librealsense2/rs.hpp> // Include RealSense Cross-Platform API
#include <opencv2/opencv.hpp>   // For saving images (optional)
#include <iostream>
#include <vector>

int main() {
    try {
        // Create a RealSense pipeline
        rs2::pipeline pipe;
        rs2::config cfg;

        // Enable depth stream
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        // Start the pipeline with the configuration
        pipe.start(cfg);

        std::cout << "Streaming depth images. Press CTRL+C to stop.\n";

        while (true) {
            // Wait for the next set of frames
            rs2::frameset frames = pipe.wait_for_frames();

            // Get the depth frame
            rs2::depth_frame depth = frames.get_depth_frame();

            // Get dimensions of the depth frame
            int width = depth.get_width();
            int height = depth.get_height();

            // Convert the depth frame to an OpenCV matrix
            cv::Mat depth_image(cv::Size(width, height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

            // Display depth image
            cv::imshow("Depth Image", depth_image);
            if (cv::waitKey(1) == 27) // Stop if 'ESC' is pressed
                break;

            // Optionally, save the depth image as a PNG file
            // cv::imwrite("depth_image.png", depth_image);
        }
    } catch (const rs2::error &e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "("
                  << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

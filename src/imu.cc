#include <sl/Camera.hpp>
#include <iostream>
#include <chrono>
#include <thread>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ------------------------------
// State and EKF classes
// ------------------------------

struct State {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    // We store orientation as a quaternion. 
    Eigen::Quaterniond orientation;
};

class InertialEKF {
public:
    // 6D state: [px, py, pz, vx, vy, vz]
    // Covariances are 6x6.
    InertialEKF() {
        // Initialize the state vector
        x = Eigen::VectorXd::Zero(6);
        
        // Initialize covariance
        P = Eigen::MatrixXd::Identity(6, 6) * 1e-3;
        
        // Process noise (tune for your system)
        Q = Eigen::MatrixXd::Identity(6, 6) * 1e-4;
        
        // Measurement noise for ZUPT (velocity = 0)
        R = Eigen::MatrixXd::Identity(3, 3) * 1e-2;
    }
    
    // Predict step
    // acc_world: acceleration in world frame (already oriented and gravity-compensated if desired)
    // dt: time step
    void predict(const Eigen::Vector3d& acc_world, double dt) {
        // Current state
        // x(0,1,2) = position
        // x(3,4,5) = velocity

        // 1. Predict the new state with a simple constant-acceleration model
        Eigen::VectorXd x_pred = x;
        x_pred.segment<3>(0) += x.segment<3>(3) * dt + 0.5 * acc_world * dt * dt; // p = p + v*dt + 0.5*a*dt^2
        x_pred.segment<3>(3) += acc_world * dt;                                  // v = v + a*dt

        // 2. Compute the Jacobian F of the state transition
        // For simplicity, F is mostly identity plus the partial derivatives wrt velocity
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6,6);
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        
        // 3. Propagate covariance
        P = F * P * F.transpose() + Q;
        
        // 4. Update state
        x = x_pred;
    }
    
    // Update step (Zero-Velocity Update)
    // meas_vel: measured velocity in world frame (in ZUPT, it's [0,0,0])
    void updateZUPT(const Eigen::Vector3d& meas_vel) {
        // Measurement model: z = v
        // H = [0_3x3 | I_3x3]
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,6);
        H.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
        
        // Predicted velocity
        Eigen::Vector3d v_pred = x.segment<3>(3);
        
        // Innovation
        Eigen::Vector3d y = meas_vel - v_pred; // meas_vel is expected to be ~0 if stationary
        
        // Innovation covariance
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        
        // Kalman gain
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();
        
        // State update
        x = x + K * y;
        
        // Covariance update
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
        P = (I - K * H) * P;
    }
    
    // Get the estimated state
    Eigen::Vector3d getPosition() const {
        return x.segment<3>(0);
    }
    
    Eigen::Vector3d getVelocity() const {
        return x.segment<3>(3);
    }
    
    // Set the initial state
    void setState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) {
        x.segment<3>(0) = pos;
        x.segment<3>(3) = vel;
    }
    
private:
    Eigen::VectorXd x;       // [px, py, pz, vx, vy, vz]
    Eigen::MatrixXd P;       // Covariance 6x6
    Eigen::MatrixXd Q;       // Process noise 6x6
    Eigen::MatrixXd R;       // Measurement noise for velocity (3x3)
};

// ------------------------------
// Utility: Stationary detector
// Very rough approach; refine for real usage!
// ------------------------------
bool isStationary(const Eigen::Vector3d& angular_velocity_deg_s,
                  const Eigen::Vector3d& linear_accel_m_s2,
                  double gyro_threshold_deg_s = 1.0,
                  double accel_threshold_m_s2 = 0.1) {
    // Check if the absolute values are below thresholds
    // This is a naive approach!
    if ((std::fabs(angular_velocity_deg_s.x()) < gyro_threshold_deg_s) &&
        (std::fabs(angular_velocity_deg_s.y()) < gyro_threshold_deg_s) &&
        (std::fabs(angular_velocity_deg_s.z()) < gyro_threshold_deg_s) &&
        (linear_accel_m_s2.norm() < (9.81 + accel_threshold_m_s2)) &&
        (linear_accel_m_s2.norm() > (9.81 - accel_threshold_m_s2))) {
        return true;
    }
    return false;
}

// ------------------------------
// Main
// ------------------------------
int main(int argc, char** argv) {
    // ----------------------
    // Initialize ZED camera
    // ----------------------
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.depth_mode = sl::DEPTH_MODE::NONE; 
    init_params.coordinate_units = sl::UNIT::METER; 
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; 

    auto returned_state = zed.open(init_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error " << returned_state << ", exit program." << std::endl;
        return -1;
    }

    if (!zed.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::GYROSCOPE)) {
        std::cerr << "IMU data is not available on this camera model." << std::endl;
        zed.close();
        return -1;
    }

    // ----------------------
    // Prepare EKF
    // ----------------------
    InertialEKF ekf;
    State state;
    state.position = Eigen::Vector3d::Zero();
    state.velocity = Eigen::Vector3d::Zero();
    // Identity orientation (no rotation)
    state.orientation = Eigen::Quaterniond::Identity();

    ekf.setState(state.position, state.velocity);

    // Gravity in Right-handed Y-up
    // In ZED's RIGHT_HANDED_Y_UP system, gravity is negative in Y
    Eigen::Vector3d gravity(0, -9.81, 0);

    // For time step calculation
    auto previous_time = std::chrono::steady_clock::now();

    sl::SensorsData sensors_data;

    // Main loop
    while (true) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT);

            // Retrieve IMU data
            auto angular_velocity = sensors_data.imu.angular_velocity;   // [deg/s]
            auto linear_acceleration = sensors_data.imu.linear_acceleration; // [m/s^2]
            auto zed_orientation = sensors_data.imu.pose.getOrientation();

            // Convert ZED orientation to Eigen Quaternion
            // ZED orientation = (ox, oy, oz, ow)
            // Notice that ZED uses a float-based struct. Convert to double:
            Eigen::Quaterniond q;
            q.w() = static_cast<double>(zed_orientation.ow);
            q.x() = static_cast<double>(zed_orientation.ox);
            q.y() = static_cast<double>(zed_orientation.oy);
            q.z() = static_cast<double>(zed_orientation.oz);

            // Update our state's orientation with the ZED orientation
            state.orientation = q.normalized();

            // Time step
            auto current_time = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time - previous_time).count();
            previous_time = current_time;

            // Transform linear acceleration to world frame using the orientation
            // Body-frame acceleration -> World-frame: a_world = q * a_body
            // If the linear_acceleration from the ZED is already in the camera reference frame,
            // we must apply the orientation to get it into the world frame.
            Eigen::Vector3d acc_body(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
            Eigen::Vector3d acc_world = state.orientation * acc_body;

            // Subtract gravity (because the ZED's linear_acceleration might include gravity).
            // Depending on the ZED's IMU data format, you may need to verify if it's "raw" or "gravity-compensated."
            // The docs usually state it's "raw" + gravity. If so, we remove gravity:
            acc_world -= gravity;

            // Perform EKF prediction
            ekf.predict(acc_world, dt);

            // Check if stationary (naive approach)
            Eigen::Vector3d gyro_deg_s(angular_velocity.x, angular_velocity.y, angular_velocity.z);
            bool stationary = isStationary(gyro_deg_s, acc_body);

            // If stationary, perform a zero-velocity update
            if (stationary) {
                ekf.updateZUPT(Eigen::Vector3d::Zero());
            }

            // Update our state (position, velocity) from the EKF
            state.position = ekf.getPosition();
            state.velocity = ekf.getVelocity();

            // Print debug info
            std::cout << "---------------------------------\n";
            std::cout << "dt: " << dt << " sec\n";
            std::cout << "Angular Velocity [deg/s]: "
                      << angular_velocity.x << ", "
                      << angular_velocity.y << ", "
                      << angular_velocity.z << std::endl;

            std::cout << "Linear Accel [m/s^2] (body): "
                      << linear_acceleration.x << ", "
                      << linear_acceleration.y << ", "
                      << linear_acceleration.z << std::endl;

            std::cout << "Orientation (ZED Quaternion): "
                      << q.x() << ", "
                      << q.y() << ", "
                      << q.z() << ", "
                      << q.w() << std::endl;

            std::cout << "EKF Position [m]: "
                      << state.position.transpose() << std::endl;
            std::cout << "EKF Velocity [m/s]: "
                      << state.velocity.transpose() << std::endl;

        } else {
            std::cerr << "Failed to grab data." << std::endl;
            // Sleep a bit before retrying
            sl::sleep_ms(10);
            continue;
        }

        // Sleep some ms if desired
        sl::sleep_ms(10);
    }

    zed.close();
    return 0;
}

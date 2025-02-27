#include <sl/Camera.hpp>
#include <iostream>
#include <chrono>
#include <thread>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>


struct State {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    // We store orientation as a quaternion. 
    Eigen::Quaterniond orientation;
};

class InertialEKF {
public:
    InertialEKF() {
        x = Eigen::VectorXd::Zero(6);
        
        P = Eigen::MatrixXd::Identity(6, 6) * 1e-3;
        
        Q = Eigen::MatrixXd::Identity(6, 6) * 1e-4;
        
        R = Eigen::MatrixXd::Identity(3, 3) * 1e-2;
    }
    

    void predict(const Eigen::Vector3d& acc_world, double dt) {

        Eigen::VectorXd x_pred = x;
        x_pred.segment<3>(0) += x.segment<3>(3) * dt + 0.5 * acc_world * dt * dt; 
        x_pred.segment<3>(3) += acc_world * dt;                                
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6,6);
        F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
        
        P = F * P * F.transpose() + Q;
        
        x = x_pred;
    }
    

    void updateZUPT(const Eigen::Vector3d& meas_vel) {

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,6);
        H.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
        

        Eigen::Vector3d v_pred = x.segment<3>(3);
        

        Eigen::Vector3d y = meas_vel - v_pred; 
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        x = x + K * y;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
        P = (I - K * H) * P;
    }

    Eigen::Vector3d getPosition() const {
        return x.segment<3>(0);
    }
    
    Eigen::Vector3d getVelocity() const {
        return x.segment<3>(3);
    }

    void setState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel) {
        x.segment<3>(0) = pos;
        x.segment<3>(3) = vel;
    }
    
private:
    Eigen::VectorXd x;      
    Eigen::MatrixXd P;      
    Eigen::MatrixXd Q;      
    Eigen::MatrixXd R;      
};


bool isStationary(const Eigen::Vector3d& angular_velocity_deg_s,
                  const Eigen::Vector3d& linear_accel_m_s2,
                  double gyro_threshold_deg_s = 1.0,
                  double accel_threshold_m_s2 = 0.1) {

    if ((std::fabs(angular_velocity_deg_s.x()) < gyro_threshold_deg_s) &&
        (std::fabs(angular_velocity_deg_s.y()) < gyro_threshold_deg_s) &&
        (std::fabs(angular_velocity_deg_s.z()) < gyro_threshold_deg_s) &&
        (linear_accel_m_s2.norm() < (9.81 + accel_threshold_m_s2)) &&
        (linear_accel_m_s2.norm() > (9.81 - accel_threshold_m_s2))) {
        return true;
    }
    return false;
}

int main(int argc, char** argv) {

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


    InertialEKF ekf;
    State state;
    state.position = Eigen::Vector3d::Zero();
    state.velocity = Eigen::Vector3d::Zero();
    state.orientation = Eigen::Quaterniond::Identity();

    ekf.setState(state.position, state.velocity);

    Eigen::Vector3d gravity(0, -9.81, 0);


    auto previous_time = std::chrono::steady_clock::now();

    sl::SensorsData sensors_data;

    while (true) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT);


            auto angular_velocity = sensors_data.imu.angular_velocity;   // [deg/s]
            auto linear_acceleration = sensors_data.imu.linear_acceleration; // [m/s^2]
            auto zed_orientation = sensors_data.imu.pose.getOrientation();


            Eigen::Quaterniond q;
            q.w() = static_cast<double>(zed_orientation.ow);
            q.x() = static_cast<double>(zed_orientation.ox);
            q.y() = static_cast<double>(zed_orientation.oy);
            q.z() = static_cast<double>(zed_orientation.oz);


            state.orientation = q.normalized();


            auto current_time = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time - previous_time).count();
            previous_time = current_time;


            Eigen::Vector3d acc_body(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
            Eigen::Vector3d acc_world = state.orientation * acc_body;


            acc_world -= gravity;


            ekf.predict(acc_world, dt);


            Eigen::Vector3d gyro_deg_s(angular_velocity.x, angular_velocity.y, angular_velocity.z);
            bool stationary = isStationary(gyro_deg_s, acc_body);


            if (stationary) {
                ekf.updateZUPT(Eigen::Vector3d::Zero());
            }


            state.position = ekf.getPosition();
            state.velocity = ekf.getVelocity();


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

            sl::sleep_ms(10);
            continue;
        }


        sl::sleep_ms(10);
    }

    zed.close();
    return 0;
}

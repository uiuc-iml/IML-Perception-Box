#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Variables
EIGEN_VERSION="3.3.7"
OPENCV_VERSION="4.5.5"
FBoW_REPO="https://github.com/stella-cv/FBoW.git"
G2O_REPO="https://github.com/RainerKuemmerle/g2o.git"
G2O_CHECKOUT="20230223_git"
BACKWARD_CPP_REPO="https://github.com/bombela/backward-cpp.git"
BACKWARD_CPP_CHECKOUT="5ffb2c879ebdbea3bdb8477c671e32b1c984beaa"
PANGOLIN_REPO="https://github.com/stevenlovegrove/Pangolin.git"
PANGOLIN_CHECKOUT="eab3d3449a33a042b1ee7225e1b8b593b1b21e3e"
STELLA_VSLAM_REPO="https://github.com/stella-cv/stella_vslam.git"
PANGOLIN_VIEWER_REPO="https://github.com/stella-cv/pangolin_viewer.git"
#PERCEPTION_BOX_REPO="https://github.com/uiuc-iml/Perception-Box.git"
ZED_SDK_REPO="https://download.stereolabs.com/zedsdk/4.0/jp54/jetsons"
REALSENSE_REPO="https://github.com/IntelRealSense/librealsense.git"


# Function to install system dependencies
install_dependencies() {
    echo "Updating system packages..."
    sudo apt update
    sudo apt upgrade -y --no-install-recommends

    echo "Installing basic dependencies..."
    sudo apt install -y build-essential pkg-config cmake git wget curl unzip

    echo "Installing g2o dependencies..."
    sudo apt install -y libatlas-base-dev libsuitesparse-dev

    echo "Installing OpenCV dependencies..."
    sudo apt install -y libgtk-3-dev ffmpeg libavcodec-dev libavformat-dev \
    libavutil-dev libswscale-dev libavresample-dev libtbb-dev

    echo "Installing Eigen dependencies..."
    sudo apt install -y gfortran

    echo "Installing backward-cpp dependencies (optional but recommended)..."
    sudo apt install -y binutils-dev

    echo "Installing other dependencies..."
    sudo apt install -y libyaml-cpp-dev libgflags-dev sqlite3 libsqlite3-dev

    echo "Installing Pangolin dependencies..."
    sudo apt install -y libglew-dev
}

# Function to install Eigen
install_eigen() {
    echo "Installing Eigen ${EIGEN_VERSION}..."
    cd ~/Downloads
    wget -q https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.bz2
    tar xf eigen-${EIGEN_VERSION}.tar.bz2
    cd eigen-${EIGEN_VERSION}
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
    make -j$(nproc)
    sudo make install
}

# Function to install OpenCV
install_opencv() {
    echo "Installing OpenCV ${OPENCV_VERSION}..."
    cd ~/Downloads
    wget -q https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip
    unzip -q ${OPENCV_VERSION}.zip
    cd opencv-${OPENCV_VERSION}
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local \
          -DBUILD_DOCS=OFF \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_TESTS=OFF \
          -DBUILD_PERF_TESTS=OFF \
          -DBUILD_opencv_python_bindings_generator=OFF \
          -DWITH_TBB=ON \
          -DWITH_OPENMP=ON \
          -DWITH_FFMPEG=ON \
          -DWITH_GTK=ON \
          -DWITH_V4L=ON \
          -DWITH_OPENGL=ON \
          -DWITH_GSTREAMER=ON \
          -DENABLE_FAST_MATH=ON \
          -DWITH_CUDA=ON \
          ..
    make -j$(nproc)
    sudo make install
}

# Function to install FBoW
install_fbow() {
    echo "Installing FBoW..."
    cd ~/Downloads
    git clone ${FBoW_REPO}
    cd FBoW
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
    make -j$(nproc)
    sudo make install
}

# Function to install g2o
install_g2o() {
    echo "Installing g2o..."
    cd ~/Downloads
    git clone ${G2O_REPO}
    cd g2o
    git checkout ${G2O_CHECKOUT}
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local \
          -DBUILD_SHARED_LIBS=ON \
          -DBUILD_UNITTESTS=OFF \
          -DG2O_USE_CHOLMOD=OFF \
          -DG2O_USE_CSPARSE=ON \
          -DG2O_USE_OPENGL=OFF \
          -DG2O_USE_OPENMP=OFF \
          -DG2O_BUILD_APPS=OFF \
          -DG2O_BUILD_EXAMPLES=OFF \
          -DG2O_BUILD_LINKED_APPS=OFF \
          ..
    make -j$(nproc)
    sudo make install
}

# Function to install backward-cpp
install_backward_cpp() {
    echo "Installing backward-cpp..."
    cd ~/Downloads
    git clone ${BACKWARD_CPP_REPO}
    cd backward-cpp
    git checkout ${BACKWARD_CPP_CHECKOUT}
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
    make -j$(nproc)
    sudo make install
}

# Function to install Pangolin
install_pangolin() {
    echo "Installing Pangolin..."
    cd ~/Downloads
    git clone ${PANGOLIN_REPO}
    cd Pangolin
    git checkout ${PANGOLIN_CHECKOUT}
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_PANGOLIN_PYTHON=OFF \
          ..
    make -j$(nproc)
    sudo make install
}

# Function to build and install Stella-VSLAM
build_stella_vslam() {
    echo "Building and installing Stella-VSLAM..."
    mkdir -p ~/stella_ws/src
    cd ~/stella_ws/src
    git clone --recursive ${STELLA_VSLAM_REPO}
    cd stella_vslam
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
    make -j$(nproc)
    sudo make install
}

# Function to build and install Pangolin Viewer
build_pangolin_viewer() {
    echo "Building and installing Pangolin Viewer..."
    cd ~/stella_ws/src
    git clone --recursive ${PANGOLIN_VIEWER_REPO}
    cd pangolin_viewer
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
    make -j$(nproc)
    sudo make install
}

# Function to build Perception Box
build_perception_box() {
    echo "Building Perception Box..."
    cd ~/Downloads
    cd Perception-Box
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
}



# Install Intel RealSense SDK
install_realsense() {
    echo "Installing Intel RealSense SDK..."
    cd ~/Downloads
    git clone ${REALSENSE_REPO}
    cd librealsense
    mkdir build && cd build
    cmake ../ -DCMAKE_BUILD_TYPE=Release \
              -DBUILD_EXAMPLES=false \
              -DFORCE_RSUSB_BACKEND=true
    make -j$(nproc)
    sudo make install
}

# Install ZED SDK
install_zed() {
    echo "Installing ZED SDK..."
    cd ~/Downloads
    wget ${ZED_SDK_REPO}/ZED_SDK_Linux_JP54_v4.0.8.run
    chmod +x ZED_SDK_Linux_JP54_v4.0.8.run
    ./ZED_SDK_Linux_JP54_v4.0.8.run -- silent
}

# Main execution flow
main() {
    echo "Starting installation process..."

    install_dependencies
    install_eigen
    install_opencv
    install_fbow
    install_g2o

    # backward-cpp is optional but recommended
    read -p "Do you want to install backward-cpp? (y/n) [y]: " install_backward
    install_backward=${install_backward:-y}
    if [[ "$install_backward" =~ ^[Yy]$ ]]; then
        install_backward_cpp
    else
        echo "Skipping backward-cpp installation."
    fi

    install_pangolin
    build_stella_vslam
    build_pangolin_viewer
    read -p "Install Intel RealSense SDK? (y/n) [y]: " install_rs
    install_rs=${install_rs:-y}
    if [[ "$install_rs" =~ ^[Yy]$ ]]; then
        install_realsense
    fi

    read -p "Install ZED SDK? (y/n) [y]: " install_zed
    install_zed=${install_zed:-y}
    if [[ "$install_zed" =~ ^[Yy]$ ]]; then
        install_zed
    fi
    build_perception_box

    echo "Installation and setup completed successfully!"
}


# Run the main function
main

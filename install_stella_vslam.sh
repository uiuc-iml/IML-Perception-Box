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
REALSENSE_REPO="https://github.com/IntelRealSense/librealsense.git"

# Function to install system dependencies
install_dependencies() {
    echo "Updating system packages..."
    sudo apt update && sudo apt upgrade -y --no-install-recommends

    echo "Installing basic dependencies..."
    sudo apt install -y build-essential pkg-config cmake git wget curl unzip

    echo "Installing g2o dependencies..."
    sudo apt install -y libatlas-base-dev libsuitesparse-dev

    echo "Installing OpenCV dependencies..."
    sudo apt install -y libgtk-3-dev ffmpeg libavcodec-dev libavformat-dev \
    libavutil-dev libswscale-dev libswresample-dev libtbb-dev

    echo "Installing Eigen dependencies..."
    sudo apt install -y gfortran

    echo "Installing backward-cpp dependencies..."
    sudo apt install -y binutils-dev

    echo "Installing other dependencies..."
    sudo apt install -y libyaml-cpp-dev libgflags-dev sqlite3 libsqlite3-dev libglew-dev
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
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_DOCS=OFF -DBUILD_EXAMPLES=OFF \
          -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DWITH_TBB=ON -DWITH_OPENMP=ON -DWITH_FFMPEG=ON -DWITH_GTK=ON \
          -DWITH_V4L=ON -DWITH_OPENGL=ON -DWITH_GSTREAMER=ON -DENABLE_FAST_MATH=ON -DWITH_CUDA=ON ..
    make -j$(nproc)
    sudo make install
}

# Function to install FBoW
declare -a REPOS=(${FBoW_REPO} ${G2O_REPO} ${BACKWARD_CPP_REPO} ${PANGOLIN_REPO})
install_git_repo() {
    repo_url=$1
    repo_name=$(basename $repo_url .git)
    echo "Installing ${repo_name}..."
    cd ~/Downloads
    [[ -d ${repo_name} ]] || git clone ${repo_url}
    cd ${repo_name}
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
    make -j$(nproc)
    sudo make install
}

# Function to install Intel RealSense SDK
install_realsense() {
    echo "Installing Intel RealSense SDK..."
    cd ~/Downloads
    git clone ${REALSENSE_REPO}
    cd librealsense
    mkdir -p build && cd build
    cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DFORCE_RSUSB_BACKEND=true
    make -j$(nproc)
    sudo make install
}

# Main execution flow
main() {
    echo "Starting installation process..."
    install_dependencies
    install_eigen
    install_opencv
    for repo in "${REPOS[@]}"; do
        install_git_repo ${repo}
    done

    read -p "Do you want to install Intel RealSense SDK? (y/n) [y]: " install_rs
    install_rs=${install_rs:-y}
    if [[ "$install_rs" =~ ^[Yy]$ ]]; then
        install_realsense
    fi

    echo "Installation and setup completed successfully!"
}

# Run the main function
main

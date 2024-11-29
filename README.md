# stella_vslam_examples

## License

This module was originally included in xdspacelab/openvslam. Therefore, the license follows the original license of xdspacelab/openvslam (BSD 2-Clause).

The following files are derived from third-party libraries.

- `./3rd/filesystem` : [gulrak/filesystem](https://github.com/gulrak/filesystem) (MIT license)
- `./3rd/popl` : [badaix/popl \[v1.2.0\]](https://github.com/badaix/popl) (MIT license)


# Stella-VSLAM on NVIDIA Jetson with Pangolin Viewer

[Stella-VSLAM](https://github.com/stella-cv/stella_vslam) is a versatile and highly customizable SLAM framework. This guide provides step-by-step instructions to install Stella-VSLAM with Pangolin Viewer on NVIDIA Jetson devices running Ubuntu.

---

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [1. Install Dependencies](#1-install-dependencies)
  - [2. Install Required Libraries](#2-install-required-libraries)
  - [3. Build and Install Stella-VSLAM](#3-build-and-install-stella-vslam)
  - [4. Build and Install Pangolin Viewer](#4-build-and-install-pangolin-viewer)
  - [5. Build and Install Examples](#5-build-and-install-examples)
- [Usage](#usage)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Features

- **Multi-Camera Support:** Monocular, stereo, and RGB-D cameras.
- **Flexible Input:** Supports video files and live camera feeds.
- **Pangolin Viewer:** OpenGL-based viewer for real-time visualization.
- **Optimized for Embedded Systems:** Suitable for NVIDIA Jetson platforms.

---

## Prerequisites

- **Operating System:** Ubuntu 18.04 or 20.04 (for NVIDIA Jetson)
- **Compiler:** GCC with C++11 support
- **CMake:** Version 3.5 or higher
- **Git**

---

## Installation

### 1. Install Dependencies

Open a terminal and execute the following commands:

```bash
sudo apt update
sudo apt upgrade -y --no-install-recommends

# Basic dependencies
sudo apt install -y build-essential pkg-config cmake git wget curl unzip

# g2o dependencies
sudo apt install -y libatlas-base-dev libsuitesparse-dev

# OpenCV dependencies
sudo apt install -y libgtk-3-dev ffmpeg libavcodec-dev libavformat-dev \
libavutil-dev libswscale-dev libavresample-dev libtbb-dev

# Eigen dependencies
sudo apt install -y gfortran

# backward-cpp dependencies (optional but recommended)
sudo apt install -y binutils-dev

# Other dependencies
sudo apt install -y libyaml-cpp-dev libgflags-dev sqlite3 libsqlite3-dev

# Pangolin dependencies
sudo apt install -y libglew-dev
```

### 2. Install Required Libraries
Install Eigen

```bash
cd ~/Downloads
wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2
tar xf eigen-3.3.7.tar.bz2
cd eigen-3.3.7
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
```
Install OpenCV
```bash
cd ~/Downloads
# Download OpenCV source
wget -q https://github.com/opencv/opencv/archive/4.5.5.zip
unzip -q 4.5.5.zip
cd opencv-4.5.5
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
```
Install FBoW
```bash
cd ~/Downloads
git clone https://github.com/stella-cv/FBoW.git
cd FBoW
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
```

Install g2o
```bash
cd ~/Downloads
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 20230223_git
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
```
Install backward-cpp (Optional but Recommended)
```bash
cd ~/Downloads
git clone https://github.com/bombela/backward-cpp.git
cd backward-cpp
git checkout 5ffb2c879ebdbea3bdb8477c671e32b1c984beaa
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
```
Install Pangolin
```bash
cd ~/Downloads
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout eab3d3449a33a042b1ee7225e1b8b593b1b21e3e
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_PANGOLIN_PYTHON=OFF \
      ..
make -j$(nproc)
sudo make install
```
### 3. Build and Install Stella-VSLAM
```bash
mkdir -p ~/stella_ws/src
cd ~/stella_ws/src
git clone --recursive https://github.com/stella-cv/stella_vslam.git
cd stella_vslam
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j$(nproc)
sudo make install
```
### 4. Build and Install Pangolin Viewer
```bash
cd ~/stella_ws/src
git clone --recursive https://github.com/stella-cv/pangolin_viewer.git
cd pangolin_viewer
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j$(nproc)
sudo make install
```

# Perception Box
## Perception Box Overview

The **Perception Box** is an integrated framework for real-time visual-inertial SLAM, 3D semantic mapping, and indoor navigation — designed to make advanced perception algorithms as easy to use as a Python API call.

Complex spatial understanding tasks often require robotics developers and researchers to manually install, build, and configure multiple libraries and algorithms from scratch. The Perception Box removes this barrier by packaging 3D mapping, semantic segmentation, and navigation algorithms into an accessible, modular system with a simple, consistent interface.

This project lowers the entry barrier for deploying modern robotic perception capabilities on embedded systems like the NVIDIA Jetson, as well as standard Ubuntu desktops. By exposing a clear XML-RPC and Python API, it allows anyone to run live SLAM, stream labeled 3D maps, and control mapping tasks remotely — without the overhead of deep low-level integration.

## Table of Contents

- [Acknowledgments](#acknowledgments)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)

---
#### Ver 1
  - [1. Install Dependencies](#1-install-dependencies)
  - [2. Install Required Libraries](#2-install-required-libraries)
  - [3. Build and Install Stella-VSLAM](#3-build-and-install-stella-vslam)
  - [4. Build and Install Pangolin Viewer](#4-build-and-install-pangolin-viewer)
  - [5. Build Perception Box](#5-Build-Perception-Box)
#### Ver 2
  - [Bash Script Installation](#Bash-Installation)
---
- [Create Config File](#configurator)
- [Usage](#Run-Perception-Box)


---

## Acknowledgments

We extend our gratitude to our mentors and advisors for their guidance and support:

- **Professor Kris Hauser**  
- **João Marques**

This project is built as a fork of the [stella-vslam-examples](https://github.com/stella-cv/stella_vslam_examples) repository, with contributions from members of the [IML Research Team](https://iml.lab.illinois.edu/):

- **Aaditya Voruganti** ([GitHub](https://github.com/aadityavoru))
- **Vallabh Nadgir** ([GitHub](https://github.com/vallabh1))

## Features

- **Multi-Camera Support:** Monocular, and RGB-D cameras.
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

### 5. Build Perception Box

```bash
git clone https://github.com/uiuc-iml/Perception-Box.git
cd Perception-Box
mkdir build
cd build
cmake ..
```
Download the Vocab file from: [Vocab file](https://drive.google.com/file/d/158_S22qdE8fUNCVYpWLcF2e-X8t7DLLH/view?usp=sharing)

Open3D: [Jetson fork](https://github.com/henryrobbins/Open3D.git)

### Bash Installation
```bash
cd ~/Downloads
git clone "https://github.com/uiuc-iml/Perception-Box.git"
chmod +x install_stella_vslam.sh
./install_stella_vslam.sh
```
### Configurator
In python helpers, there is a configurator for the zed and realsense camera - 
Usage

    Run the Flask App
    
Access the Web App Open your browser and go to http://127.0.0.1:5000.

Configure Camera Settings

    Select the camera type (ZED or RealSense).
    Choose resolution, frame rate, and other camera options.
    Set the socket address and port.

Generate YAML

    Click the Generate YAML button.
    Download the generated camera_config.yaml file.
### Install requirements for the mapping module

```
cd Perception-Box
pip install -r requirements.txt
```

### Run Mapping
On the client device, create an XML-RPC client using the perception box's local network IP. Enter the mapping directory of the Perception-Box folder and run the mapping server on the perception box:
```
python testserver.py
```
Next, run the perception box using the script given below. Then, use the available APIs to start, pause, and end mapping from the client side. Use the get_metric_map and get_semantic_map APIs to get the map over the 
XML-RPC interface. Look at the [documentation](https://github.com/uiuc-iml/IML-Perception-Box/wiki/Mapping-Module-API-Documentation) for more information. See example_client.py for inspiration.
Note: Be sure to change the IP to your perception box's IP.

### Run Perception Box
Example:
```bash
./run_camera_slam --vocab /home/perception/lib/stella_vslam_examples/build/orb_vocab.fbow --config /home/perception/lib/stella_vslam_examples/build/realsense.yaml --number 4 --viewer pangolin_viewer
```
Note: Adjust the paths based on the local configuration of your system

### Example sequence of commands
First run the mapping box (We could make a bash script that does this on start everytime). Then, use the above bash script to start SLAM (We could make it so that it executes on start_task()). Then, call start_mapping() to start
mapping (from the client). Finally, use the APIs to get maps and call stop_mapping() and stop_task() when finished.






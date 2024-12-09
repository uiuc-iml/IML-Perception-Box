cmake_minimum_required(VERSION 3.1)
project(stella_vslam_examples LANGUAGES CXX C)

# ----- Find dependencies -----

find_package(stella_vslam REQUIRED)

# RealSense
find_package(realsense2 REQUIRED)

# ZED SDK
find_package(ZED 3 REQUIRED)

# filesystem
set(filesystem_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/filesystem/include)

# popl
set(popl_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/popl/include)

set(USE_STACK_TRACE_LOGGER OFF CACHE BOOL "Enable automatic stack trace logger of backward-cpp")
if(USE_STACK_TRACE_LOGGER)
    find_package(Backward REQUIRED)
    message(STATUS "Stack trace logger: ENABLED")
else()
    message(STATUS "Stack trace logger: DISABLED")
endif()

set(USE_GOOGLE_PERFTOOLS OFF CACHE BOOL "Enable profiler of google-perftools")
if(USE_GOOGLE_PERFTOOLS)
    find_package(Gperftools REQUIRED)
    include_directories(${GPERFTOOLS_INCLUDE_DIRS})
    message(STATUS "Google Perftools: ENABLED")
else()
    message(STATUS "Google Perftools: DISABLED")
endif()

# ----- Show dialog -----

find_package(pangolin_viewer QUIET)
if(pangolin_viewer_FOUND)
    message(STATUS "Viewer for examples: PangolinViewer")
endif()
find_package(socket_publisher QUIET)
if(socket_publisher_FOUND)
    message(STATUS "Viewer for examples: SocketPublisher")
endif()
find_package(iridescence_viewer QUIET)
if(iridescence_viewer_FOUND)
    message(STATUS "Viewer for examples: IridescenceViewer")
endif()

# ----- Build example executables -----

set(EXECUTABLE_TARGETS "")

add_executable(run_camera_slam src/run_camera_slam.cc)
list(APPEND EXECUTABLE_TARGETS run_camera_slam)

# --- add_executable(run_image_slam src/run_image_slam.cc src/util/image_util.cc)
# --- list(APPEND EXECUTABLE_TARGETS run_image_slam)

# --- add_executable(run_video_slam src/run_video_slam.cc)
# --- list(APPEND EXECUTABLE_TARGETS run_video_slam)

# --- add_executable(run_euroc_slam src/run_euroc_slam.cc src/util/euroc_util.cc)
# --- list(APPEND EXECUTABLE_TARGETS run_euroc_slam)

# --- add_executable(run_kitti_slam src/run_kitti_slam.cc src/util/kitti_util.cc)
# --- list(APPEND EXECUTABLE_TARGETS run_kitti_slam)

# --- add_executable(run_tum_rgbd_slam src/run_tum_rgbd_slam.cc src/util/tum_rgbd_util.cc)
# --- list(APPEND EXECUTABLE_TARGETS run_tum_rgbd_slam)

# --- add_executable(run_loop_closure src/run_loop_closure.cc)
# --- list(APPEND EXECUTABLE_TARGETS run_loop_closure)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)

    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
    pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
    pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
    pkg_check_modules(GOBJECT REQUIRED gobject-2.0)
    pkg_check_modules(GLIB REQUIRED glib-2.0)

    include_directories(
        ${GSTREAMER_INCLUDE_DIRS}
        ${GSTREAMER_BASE_INCLUDE_DIRS}
        ${GSTREAMER_APP_INCLUDE_DIRS}
        ${GOBJECT_INCLUDE_DIRS}
        ${GLIB_INCLUDE_DIRS}
        /usr/lib/aarch64-linux-gnu/glib-2.0/include
        ${realsense2_INCLUDE_DIR} # Add RealSense include directories
        ${ZED_INCLUDE_DIRS}       # Add ZED SDK include directories
    )

    # Link stella_vslam with keyword signature
    target_link_libraries(${EXECUTABLE_TARGET}
                        PRIVATE
                        stella_vslam::stella_vslam
                        opencv_imgcodecs
                        opencv_videoio
                        ${GSTREAMER_LIBRARIES}
                        ${GSTREAMER_BASE_LIBRARIES}
                        ${GSTREAMER_APP_LIBRARIES}
                        ${GOBJECT_LIBRARIES}
                        ${GLIB_LIBRARIES}
                        gobject-2.0
                        glib-2.0
                        ${realsense2_LIBRARY} # Link RealSense library
                        ${ZED_LIBRARIES}      # Link ZED SDK libraries
                        )

    # Set output directory for executables
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")

    # PangolinViewer is used on a priority basis
    if(pangolin_viewer_FOUND)
        # Set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE HAVE_PANGOLIN_VIEWER)
        # Link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE pangolin_viewer::pangolin_viewer)
    endif()
    if(socket_publisher_FOUND)
        # Set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE HAVE_SOCKET_PUBLISHER)
        # Link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE socket_publisher::socket_publisher)
    endif()
    if(iridescence_viewer_FOUND)
        # Set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE HAVE_IRIDESCENCE_VIEWER)
        # Link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE iridescence_viewer::iridescence_viewer)
    endif()

    # Setup stack trace logger
    if(USE_STACK_TRACE_LOGGER)
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_STACK_TRACE_LOGGER)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE Backward::Backward)
    endif()

    # Setup google-perftools
    if(USE_GOOGLE_PERFTOOLS)
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_GOOGLE_PERFTOOLS)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE ${GPERFTOOLS_LIBRARIES})
    endif()

    # Include 3rd party library headers
    target_include_directories(${EXECUTABLE_TARGET}
                               PRIVATE
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/popl/include>
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/filesystem/include>
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/spdlog/include>)
endforeach()

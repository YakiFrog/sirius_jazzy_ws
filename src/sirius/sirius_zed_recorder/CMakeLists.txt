cmake_minimum_required(VERSION 3.14)
project(sirius_zed_recorder)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc imgcodecs)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# ZED Open Capture is an official, CUDA-free USB driver for ZED/ZED Mini/ZED2/ZED2i.
# Pin the revision so robot-PC builds remain reproducible.
include(FetchContent)
FetchContent_Declare(
  zed_open_capture_source
  GIT_REPOSITORY https://github.com/stereolabs/zed-open-capture.git
  GIT_TAG 93739154903a66c112ee52c3c6fbfd83b7218617
  GIT_SHALLOW FALSE
)
FetchContent_GetProperties(zed_open_capture_source)
if(NOT zed_open_capture_source_POPULATED)
  FetchContent_Populate(zed_open_capture_source)
endif()

add_executable(zed_stereo_publisher
  src/zed_stereo_publisher.cpp
  ${zed_open_capture_source_SOURCE_DIR}/src/videocapture.cpp
)
ament_target_dependencies(zed_stereo_publisher rclcpp sensor_msgs std_msgs)
target_compile_definitions(zed_stereo_publisher PRIVATE VIDEO_MOD_AVAILABLE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(zed_stereo_publisher PRIVATE
    -Wno-deprecated-declarations
    -Wno-implicit-fallthrough
    -Wno-missing-field-initializers
  )
endif()
target_include_directories(zed_stereo_publisher PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
  ${zed_open_capture_source_SOURCE_DIR}/include
  ${zed_open_capture_source_SOURCE_DIR}/examples/include
)
target_link_directories(zed_stereo_publisher PRIVATE ${LIBUSB_LIBRARY_DIRS})
target_link_libraries(zed_stereo_publisher
  ${OpenCV_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  Threads::Threads
)

install(TARGETS zed_stereo_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

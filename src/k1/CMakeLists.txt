cmake_minimum_required(VERSION 3.16)
project(k1_pose_assistant)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# ONNX Runtime 路径可按实际安装位置调整：
# cmake -DONNXRUNTIME_ROOT=/path/to/onnxruntime ..
set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "ONNX Runtime install prefix")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${ONNXRUNTIME_ROOT}/include
    /usr/local/include
    /usr/include
)

link_directories(
    ${ONNXRUNTIME_ROOT}/lib
    /usr/local/lib
    /usr/lib
)

add_executable(pose_detector main.cpp)

target_link_libraries(pose_detector
    ${OpenCV_LIBS}
    onnxruntime
    pthread
    dl
)

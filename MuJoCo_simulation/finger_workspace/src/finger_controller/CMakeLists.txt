cmake_minimum_required(VERSION 3.16)
project(finger_controller)

set(CMAKE_CXX_STANDARD 17)

set(MUJOCO_DIR "/root/mujoco_cpp_lib")

include_directories(${MUJOCO_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR})   # so kinematics.h is visible
link_directories(${MUJOCO_DIR}/lib)

find_package(glfw3 REQUIRED)

add_executable(finger_controller finger_controller.cpp kinematics.cpp)
target_link_libraries(finger_controller
    mujoco
    glfw
    GL
    dl
    pthread
)
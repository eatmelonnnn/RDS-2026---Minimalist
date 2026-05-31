cmake_minimum_required(VERSION 3.16)
project(finger_controller)
set(CMAKE_CXX_STANDARD 17)

if(EXISTS "/opt/homebrew")
    set(HB /opt/homebrew)
else()
    set(HB /usr/local)
endif()

set(MUJOCO_FRAMEWORK_DIR "/Applications/MuJoCo.app/Contents/Frameworks")
set(MUJOCO_HEADERS "${MUJOCO_FRAMEWORK_DIR}/mujoco.framework/Headers")
set(MUJOCO_LIB "${MUJOCO_FRAMEWORK_DIR}/mujoco.framework/Versions/A/libmujoco.3.8.1.dylib")

# Create a "mujoco" symlink in the build dir so <mujoco/mujoco.h> resolves
set(MUJOCO_INCLUDE_SHIM "${CMAKE_BINARY_DIR}/mujoco_include")
file(MAKE_DIRECTORY ${MUJOCO_INCLUDE_SHIM})
if(NOT EXISTS "${MUJOCO_INCLUDE_SHIM}/mujoco")
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${MUJOCO_HEADERS} ${MUJOCO_INCLUDE_SHIM}/mujoco)
endif()

include_directories(${HB}/include)
include_directories(${MUJOCO_INCLUDE_SHIM})
include_directories(${CMAKE_SOURCE_DIR})

find_package(glfw3 REQUIRED)

# Shared linking config — applied to each executable below
set(COMMON_LIBS
    glfw
    ${MUJOCO_LIB}
    "-framework OpenGL"
    "-framework Cocoa"
    "-framework IOKit"
)

# --- finger_controller ---
add_executable(finger_controller finger_controller.cpp kinematics.cpp)
target_link_libraries(finger_controller ${COMMON_LIBS})
set_target_properties(finger_controller PROPERTIES
    BUILD_RPATH "${MUJOCO_FRAMEWORK_DIR}/mujoco.framework/Versions/A"
    INSTALL_RPATH "${MUJOCO_FRAMEWORK_DIR}/mujoco.framework/Versions/A"
)

# --- pendulum_test ---
add_executable(pendulum_test pendulum_test.cpp kinematics.cpp)
target_link_libraries(pendulum_test ${COMMON_LIBS})
set_target_properties(pendulum_test PROPERTIES
    BUILD_RPATH "${MUJOCO_FRAMEWORK_DIR}/mujoco.framework/Versions/A"
    INSTALL_RPATH "${MUJOCO_FRAMEWORK_DIR}/mujoco.framework/Versions/A"
)
cmake_minimum_required(VERSION 3.18)

project(DreamSimNative LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# compile_commands.json for VS Code IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# robot-core's portable MPC/ALS library (robotcore_mpc target)
add_subdirectory(robotcore_mpc/third_party/robotcore/jetson-nano robotcore)

# dumps ALS path samples from a JerryIO file as CSV (used by scripts/draw_als_path.py)
add_executable(als_dump tools/als_dump.cpp)
target_link_libraries(als_dump PRIVATE robotcore_mpc)

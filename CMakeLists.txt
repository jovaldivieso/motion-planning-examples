cmake_minimum_required(VERSION 3.16)
project(motion_planning_examples LANGUAGES CXX)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# Strict C++17 configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find required dependencies
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

add_executable(
    plan
    src/main.cpp
    src/two_dof_planar_arm.cpp
    src/ompl_planner.cpp
    src/ceres_planner.cpp
    src/fcl_collision_checker.cpp
)

# Enable strict compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(plan PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(plan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(plan PRIVATE 
    ompl::ompl
    Ceres::ceres
    fcl
    Eigen3::Eigen
)

# Copy runtime assets to the build directory
configure_file(config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config.yaml COPYONLY)

cmake_minimum_required(VERSION 3.20)

project(rmcs_localization)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 OLD)
    set(CMAKE_POLICY_DEFAULT_CMP0144 OLD)
endif()
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
    set(CMAKE_POLICY_DEFAULT_CMP0167 OLD)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

set(BUILD_HELPER ON CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_PCL ON CACHE BOOL "" FORCE)
set(BUILD_WITH_TBB OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_FAST_GICP OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    small_gicp
    GIT_REPOSITORY https://github.com/koide3/small_gicp.git
    GIT_TAG v1.0.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(small_gicp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(small_gicp PRIVATE -w)
    target_include_directories(small_gicp SYSTEM PUBLIC
        $<BUILD_INTERFACE:${small_gicp_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation registration)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenMP QUIET)
if(OpenMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

add_executable(rmcs_localization
    src/main.cc
    src/registration/engine.cpp
    src/runtime/runtime.cpp
    src/util/segmentation.cpp
)

target_include_directories(rmcs_localization PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(rmcs_localization
    geometry_msgs
    pcl_conversions
    pcl_ros
    rclcpp
    sensor_msgs
    std_srvs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
)

target_link_libraries(rmcs_localization
    Eigen3::Eigen
    small_gicp
    ${PCL_LIBRARIES}
)

if(OpenMP_FOUND)
    target_link_libraries(rmcs_localization OpenMP::OpenMP_CXX)
endif()

install(TARGETS rmcs_localization
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

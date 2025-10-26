cmake_minimum_required(VERSION 3.5)
project(river2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "RELEASE")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -Wno-unused-function")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -Wno-unused-function")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        rosbag
        roscpp
        rospy
        sensor_msgs
        std_msgs
        sbg_driver
        ti_mmwave_rospkg
        message_generation
        geometry_msgs
        pcl_ros
        pcl_conversions
)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        RiverState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES rivers
        CATKIN_DEPENDS rosbag roscpp rospy sensor_msgs std_msgs ti_mmwave_rospkg message_runtime
        #  DEPENDS system_lib
)


###########
## Build ##
###########
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus)
find_package(Ceres)
find_package(magic_enum)
find_package(yaml-cpp)
find_package(spdlog)

#define PCL_NO_PRECOMPILE [it's very important for Custom pcl Point type]
add_definitions(-DPCL_NO_PRECOMPILE)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ctraj/tiny CTRAJ_TINY_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ctraj/core CTRAJ_CORE_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/core CORE_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/config CONFIG_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/sensor SENSOR_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/view VIEW_SRC_FILES)

add_library(
        ${PROJECT_NAME} SHARED
        # source files
        ${CTRAJ_TINY_SRC_FILES}
        ${CTRAJ_CORE_SRC_FILES}
        ${CORE_SRC_FILES}
        ${CONFIG_SRC_FILES}
        ${SENSOR_SRC_FILES}
        ${VIEW_SRC_FILES}
)

target_include_directories(
        ${PROJECT_NAME} PUBLIC
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS} 
        ${PCL_INCLUDE_DIRS}

        magic_enum::magic_enum
        ${CMAKE_SOURCE_DIR}/../devel/include
        ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME} PUBLIC
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        spdlog::spdlog

        ${PCL_LIBRARIES}
        ${Pangolin_LIBRARIES}
        pthread
        Sophus::Sophus
        ${CERES_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_prog exe/main.cpp)
add_executable(${PROJECT_NAME}_simu exe/simulation.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
        ${PROJECT_NAME}_prog
        ${PROJECT_NAME}
)
target_link_libraries(
        ${PROJECT_NAME}_simu
        ${PROJECT_NAME}
)
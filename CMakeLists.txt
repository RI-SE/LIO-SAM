cmake_minimum_required(VERSION 3.5)
project(lio_sam)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(oxts_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find libpcap for PCAP file reading
find_package(PkgConfig REQUIRED)
pkg_check_modules(PCAP REQUIRED libpcap)

include_directories(
  include/lio_sam
  ${PCL_INCLUDE_DIRS}
  /opt/ros/humble/include
  /opt/ros/humble/include/pcl_conversions
  ${PCAP_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)
get_property(cpp_typesupport_target GLOBAL PROPERTY ROSIDL_TYPESUPPORT_CPP_${PROJECT_NAME})

# Existing executables...
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
rosidl_target_interfaces(${PROJECT_NAME}_featureExtraction ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs pcl_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
rosidl_target_interfaces(${PROJECT_NAME}_imageProjection ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_mapOptimization src/mapOptmization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM oxts_msgs geodesy Eigen) 
target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam)
rosidl_target_interfaces(${PROJECT_NAME}_mapOptimization ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_offlineProcessor src/OfflineLioSamProcessor.cpp)
ament_target_dependencies(${PROJECT_NAME}_offlineProcessor 
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs 
  pcl_conversions pcl_msgs visualization_msgs 
  tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs 
  OpenCV PCL GTSAM rosbag2_cpp rosbag2_storage)
target_link_libraries(${PROJECT_NAME}_offlineProcessor gtsam "${cpp_typesupport_target}")

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/offline_processor)

# Get the absolute path to the OXTS NComRxC files - UPDATED PATHS
set(NCOMRXC_C "${CMAKE_CURRENT_SOURCE_DIR}/src/graphSLAM/oxts_ros2_driver/oxts_driver/src/lib/NComRxC.c")
set(NCOMRXC_H "${CMAKE_CURRENT_SOURCE_DIR}/src/graphSLAM/oxts_ros2_driver/oxts_driver/include/oxts_driver/NComRxC.h")

# Check if the NComRxC files exist
if(NOT EXISTS "${NCOMRXC_C}")
  message(WARNING "NComRxC.c not found at ${NCOMRXC_C}")
endif()

if(NOT EXISTS "${NCOMRXC_H}")
  message(WARNING "NComRxC.h not found at ${NCOMRXC_H}")
endif()

# Copy NComRxC files to offline_processor directory for easier inclusion
if(EXISTS "${NCOMRXC_C}" AND EXISTS "${NCOMRXC_H}")
  configure_file(${NCOMRXC_C} ${CMAKE_CURRENT_SOURCE_DIR}/offline_processor/NComRxC.c COPYONLY)
  configure_file(${NCOMRXC_H} ${CMAKE_CURRENT_SOURCE_DIR}/offline_processor/NComRxC.h COPYONLY)
  set(HAVE_NCOMRXC TRUE)
  message(STATUS "NComRxC files found and copied to offline_processor directory")
else()
  set(HAVE_NCOMRXC FALSE)
  message(WARNING "NComRxC files not found. Creating placeholder files.")
  file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/offline_processor/NComRxC.c "// Placeholder for NComRxC.c\n")
  file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/offline_processor/NComRxC.h "// Placeholder for NComRxC.h\n")
endif()

set(OFFLINE_CORE_SOURCES
  offline_processor/offline_main.cpp
  offline_processor/offline_Processor.cpp
  offline_processor/offline_ImageProjection.cpp
  offline_processor/offline_FeatureExtraction.cpp
  offline_processor/offline_MapOptimization.cpp
  offline_processor/pcl_custom_instantiations.cpp
  offline_processor/VelodyneProcessor.cpp  # Add this line
)

# Add NComRxC only if we found it
if(HAVE_NCOMRXC)
  list(APPEND OFFLINE_CORE_SOURCES offline_processor/NComRxC.c)
endif()

foreach(SRC_FILE ${OFFLINE_CORE_SOURCES})
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${SRC_FILE})
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/${SRC_FILE} "// TODO: Implement this file\n")
  endif()
endforeach()

add_executable(${PROJECT_NAME}_offline_integrated 
  ${OFFLINE_CORE_SOURCES}
)

target_include_directories(${PROJECT_NAME}_offline_integrated PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/offline_processor
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_offline_integrated 
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs 
  pcl_conversions pcl_msgs visualization_msgs 
  tf2 tf2_eigen tf2_sensor_msgs tf2_geometry_msgs 
  OpenCV PCL GTSAM rosbag2_cpp rosbag2_storage
)

target_link_libraries(${PROJECT_NAME}_offline_integrated
  gtsam
  "${cpp_typesupport_target}"
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCAP_LIBRARIES}
  yaml-cpp  # Add this line
)

# Add preprocessor definition if we have NComRxC
if(HAVE_NCOMRXC)
  target_compile_definitions(${PROJECT_NAME}_offline_integrated PRIVATE HAVE_NCOMRXC)
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS 
    ${PROJECT_NAME}_imageProjection
    ${PROJECT_NAME}_featureExtraction
    ${PROJECT_NAME}_mapOptimization
    ${PROJECT_NAME}_offlineProcessor
    ${PROJECT_NAME}_offline_integrated
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
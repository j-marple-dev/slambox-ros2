cmake_minimum_required(VERSION 3.5)
project(slambox_ros2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

## Compile as C++17, supported in ROS Kinetic and newer
# add_compile_options(-std=c++17 -O3)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_EXTENSIONS OFF)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(glog 0.6.0 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(yaml-cpp REQUIRED )
find_package(rclcpp_components REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(slambox_sdk 0.2.0 REQUIRED)

include_directories(
  include
)
include_directories(${YAML_CPP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB SOURCES_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# Remove executable cpp files
FOREACH(source_executable ${SOURCES_EXECUTABLE})
    list(REMOVE_ITEM SOURCES ${source_executable})
ENDFOREACH()

# Add executable list of executable cpp files
FOREACH(source_executable ${SOURCES_EXECUTABLE})
  get_filename_component(source_executable_name ${source_executable} NAME_WLE)
  set(executable_name ${PROJECT_NAME}-${source_executable_name})

  add_executable( ${executable_name}
      ${SOURCES}
      ${source_executable}
  )

  target_link_libraries(${executable_name}
    glog::glog
    ${slambox_sdk_LIBRARIES}
    ZLIB::ZLIB
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  target_include_directories(${executable_name} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include
    $<INSTALL_INTERFACE:include>)
  
  ament_target_dependencies(${executable_name} rclcpp std_msgs sensor_msgs nav_msgs rclcpp_components pcl_ros pcl_conversions PCL)

  install(TARGETS
    ${executable_name}
    DESTINATION lib/${PROJECT_NAME}
)
ENDFOREACH()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_NAME_BASE test-${PROJECT_NAME})

  enable_testing()

  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  FILE(GLOB UNITTEST_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.cpp")

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  FOREACH(unittest_path ${UNITTEST_LIST})
    get_filename_component(unittest_name ${unittest_path} NAME_WLE)
    string(REPLACE "test_" "" unittest_name ${unittest_name})
    set(TEST_NAME ${TEST_NAME_BASE}-${unittest_name})

    message("Build Unit Test for ${unittest_path} with name of ${TEST_NAME}")

    ament_add_gtest(${TEST_NAME} ${unittest_path} ${SOURCES})

    ament_target_dependencies(${TEST_NAME} nav_msgs std_msgs sensor_msgs rclcpp PCL)

    target_link_libraries(${TEST_NAME}
      gtest
      gtest_main
      glog::glog
      ${slambox_sdk_LIBRARIES}
      ${YAML_CPP_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  ENDFOREACH()
endif()

ament_package()

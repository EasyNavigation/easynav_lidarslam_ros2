cmake_minimum_required(VERSION 3.20)
project(easynav_lidarslam_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(scanmatcher REQUIRED)
find_package(graph_based_slam REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(g2o REQUIRED
COMPONENTS
g2o::core
g2o::types_slam3d
g2o::solver_eigen
)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_core
  pluginlib
  geographic_msgs
  scanmatcher
  graph_based_slam
)

add_library(${PROJECT_NAME} SHARED
  src/easynav_lidarslam_localizer/LidarSlamLocalizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  easynav_core::easynav_core
  pluginlib::pluginlib
  ${geographic_msgs_TARGETS}
  GeographicLib
  ${graph_based_slam_LIBRARIES}
  ${scanmatcher_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  # scanmatcher::scanmatcher
  # graph_based_slam::graph_based_slam
  ${PCL_LIBRARIES}
  g2o::core
  g2o::types_slam3d
  g2o::solver_eigen
)
ament_target_dependencies(${PROJECT_NAME} scanmatcher graph_based_slam)
target_compile_definitions(${PROJECT_NAME} PRIVATE "GS_SM_BUILDING_DLL")


install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
# Register the localization plugins
pluginlib_export_plugin_description_file(easynav_core easynav_lidarslam_localizer_plugins.xml)
ament_export_dependencies(${dependencies})
ament_package()

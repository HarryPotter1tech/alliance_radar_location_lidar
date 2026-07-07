cmake_minimum_required(VERSION 3.24)
project(radar_lidar)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(small_gicp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search kdtree)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

# ── Core library ─────────────────────────────────────────────────
add_library(${PROJECT_NAME}_core STATIC
    src/map_data.cpp
    src/localization.cpp
    src/spherical_grid.cpp
    src/frame_accumulator.cpp
    src/dynamic_cloud.cpp
    src/cluster.cpp
)
target_link_libraries(${PROJECT_NAME}_core
    small_gicp::small_gicp
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)

# ── ROS2 node ────────────────────────────────────────────────────
ament_auto_add_executable(${PROJECT_NAME}_node
    src/pipeline.cpp
    src/runtime.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)

# ── Offline tool (CLI) ────────────────────────────────────────────
add_executable(registration_tool
    tools/registration_tool.cpp
)
target_link_libraries(registration_tool ${PROJECT_NAME}_core)

# ── Offline test node (ROS2, Foxglove) ───────────────────────────
ament_auto_add_executable(offline_test_node
    tools/offline_test_node.cpp
)
target_link_libraries(offline_test_node ${PROJECT_NAME}_core)

# ── Tests ────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(radar_tests
      test/test_map_data.cpp
      test/test_localization.cpp
      test/test_localization_roi.cpp
      test/test_detection.cpp
      test/test_offline_visualization.cpp
  )
  target_link_libraries(radar_tests ${PROJECT_NAME}_core)
endif()

install(TARGETS ${PROJECT_NAME}_node registration_tool offline_test_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()

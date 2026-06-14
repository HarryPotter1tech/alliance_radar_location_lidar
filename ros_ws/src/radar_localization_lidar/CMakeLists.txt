cmake_minimum_required(VERSION 3.14)
project(radar_localization_lidar)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -march=native -DNDEBUG -DEIGEN_NO_DEBUG -g -DRADAR_DEBUG")
add_compile_options(-flto)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(small_gicp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(assimp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
pkg_check_modules(yaml-cpp REQUIRED yaml-cpp)
find_package(hikcamera REQUIRED)

#include_directories(${hikcamera_INCLUDE_DIRS})


file(GLOB SOURCES "src/*.cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(radar_localization_lidar_node ${SOURCES})
target_link_libraries(radar_localization_lidar_node 
    ${OpenCV_LIBS}
    assimp::assimp
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)
ament_target_dependencies(radar_localization_lidar_node 
    rclcpp
    std_msgs
    sensor_msgs
    pcl_conversions
    tf2_geometry_msgs
    OpenCV
    hikcamera
    assimp
    Eigen3
    ament_index_cpp
)
  install(DIRECTORY config model
    DESTINATION ${PROJECT_NAME}
  )
install(TARGETS radar_localization_lidar_node DESTINATION lib/${PROJECT_NAME})
ament_package()

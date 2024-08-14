cmake_minimum_required(VERSION 3.8)
project(read_img)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
set(eigen3_DIR "/usr/local/share/eigen3/cmake")
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

ament_package()
include_directories(include/read_img/)
add_executable(read_rgb src/rgbd.cpp src/TUM_Datasets.cpp)
ament_target_dependencies(read_rgb rclcpp std_msgs sensor_msgs Eigen3 OpenCV cv_bridge)

install(TARGETS
  read_rgb
  DESTINATION lib/${PROJECT_NAME}
)
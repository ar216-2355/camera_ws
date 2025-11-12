cmake_minimum_required(VERSION 3.8)
project(ball_detector)

# --- コンパイラ設定 ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- ROS2依存パッケージ ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- OpenCV ---
find_package(OpenCV REQUIRED)

# --- 実行ファイル ---
add_executable(ball_detector_node src/main.cpp)
ament_target_dependencies(ball_detector_node rclcpp geometry_msgs)
target_link_libraries(ball_detector_node ${OpenCV_LIBS})

# --- インストール設定 ---
install(TARGETS
  ball_detector_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

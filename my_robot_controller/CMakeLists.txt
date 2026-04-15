cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Zależności ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# --- Budowanie Twojego pliku ---
add_executable(navigation_node src/navigation_node.cpp)

target_include_directories(navigation_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(navigation_node rclcpp std_msgs)
target_link_libraries(navigation_node ${OpenCV_LIBS})

# --- Instalacja pliku wykonywalnego ---
install(TARGETS
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Testy i koniec pliku ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
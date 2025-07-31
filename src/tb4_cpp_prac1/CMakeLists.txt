cmake_minimum_required(VERSION 3.8)
project(tb4_cpp_prac1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(irobot_create_msgs REQUIRED)

# Additional Package Dependency
find_package(std_srvs REQUIRED)

find_package(rclcpp_action REQUIRED) # added package dependency
find_package(geometry_msgs REQUIRED) # added package dependency
find_package(nav_msgs REQUIRED) # added package dependency

add_executable(tb4_cpp_node src/tb4_cpp_node.cpp)
ament_target_dependencies(tb4_cpp_node rclcpp irobot_create_msgs)

# Additional node and its dependency
add_executable(tb4_cpp_srv_server src/tb4_cpp_srv_server.cpp)
ament_target_dependencies(tb4_cpp_srv_server rclcpp irobot_create_msgs std_srvs)

add_executable(tb4_cpp_action_server src/tb4_cpp_action_server.cpp)
  ament_target_dependencies(tb4_cpp_action_server rclcpp irobot_create_msgs rclcpp_action
  geometry_msgs nav_msgs)

target_include_directories(tb4_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(tb4_cpp_srv_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(tb4_cpp_action_server PUBLIC # added specified include directory
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(tb4_cpp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS tb4_cpp_node tb4_cpp_srv_server
  DESTINATION lib/${PROJECT_NAME})

# added the new node for installation
install(TARGETS tb4_cpp_node tb4_cpp_srv_server tb4_cpp_action_server
  DESTINATION lib/${PROJECT_NAME})

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

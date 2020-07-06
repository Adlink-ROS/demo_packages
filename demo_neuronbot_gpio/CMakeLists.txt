cmake_minimum_required(VERSION 3.5)
project(demo_neuronbot_gpio)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sema_vendor REQUIRED)
if (sema_vendor_FOUND)
  find_package(libsema REQUIRED)
endif()



# For message type
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SemaGpio.srv"
)
ament_export_dependencies(rosidl_default_runtime)


# For led_sub
add_library(led_lib SHARED src/led_sub.cpp)
ament_target_dependencies(led_lib rclcpp std_msgs rclcpp_components)
target_link_libraries(led_lib semaeapi)

rclcpp_components_register_node(led_lib
  PLUGIN led_subscriber_node::LEDSubscriber
  EXECUTABLE led_sub)

install(TARGETS
  led_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# For led_pub
add_executable(led_pub src/led_pub.cpp)
ament_target_dependencies(led_pub rclcpp std_msgs)
target_link_libraries(led_pub semaeapi)

install(TARGETS
  led_pub
  DESTINATION lib/demo_neuronbot_gpio)


# For server
add_library(server_lib SHARED 
  src/sema_server.cpp)
ament_target_dependencies(server_lib
  "rclcpp"
  "rclcpp_components")
target_link_libraries(server_lib semaeapi)
rosidl_target_interfaces(server_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_node(server_lib
  PLUGIN sema_server_node::SemaServerNode
  EXECUTABLE sema_server)
install(TARGETS
  server_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# For client
add_executable(sema_client src/sema_client.cpp)
ament_target_dependencies(sema_client
  "rclcpp")
rosidl_target_interfaces(sema_client ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS sema_client
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(sirius_keyop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sirius_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

include_directories(include)

add_library(sirius_keyop SHARED src/sirius_keyop.cpp)
add_library(sirius_controller SHARED src/sirius_controller.cpp)

ament_target_dependencies(sirius_keyop
  "geometry_msgs"
  "sirius_interfaces"
  "rclcpp"
  "rclcpp_components"
)

ament_target_dependencies(sirius_controller
  "geometry_msgs"
  "std_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components" 
  "sirius_interfaces"
  "nav2_msgs"
)

rclcpp_components_register_node(sirius_keyop
  PLUGIN "sirius_keyop::KeyOp"
  EXECUTABLE sirius_keyop_node 
)

rclcpp_components_register_node(sirius_controller
  PLUGIN "sirius_controller::Controller"
  EXECUTABLE sirius_controller_node
)

install(TARGETS sirius_keyop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS sirius_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(herkulex)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Ensure rosidl_generate_interfaces runs first
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/HerkulexIjogMove.srv"
  "srv/HerkulexPositionMove.srv"
  "srv/HerkulexRegisterCommand.srv"
  "srv/HerkulexSjogMove.srv"
  "srv/HerkulexVelocityMove.srv"
  "srv/HerkulexAngleMove.srv"
  "msg/HerkulexInfoEep.msg"
  "msg/HerkulexInfoRam.msg"
  "msg/MsgHerkulexEep.msg"
  "msg/MsgHerkulexRam.msg"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# Link the generated interfaces before building executables
find_package(rosidl_default_runtime REQUIRED)

# Ensure generated headers are included
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
	${tf2_ros_INCLUDE_DIRS}

)

# Executable
add_executable(herkulex_node src/herkulex.cpp)
add_executable(herkulex_0602_ex_node src/herkulex_0602_ex.cpp) #add...0602_ex

# Set dependencies for the executable
set(dependencies
  rclcpp
  rmw_implementation
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs

)

ament_target_dependencies(herkulex_node ${dependencies})
ament_target_dependencies(herkulex_0602_ex_node ${dependencies})#add...0602_ex

# Ensure message generation completes before building the node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(herkulex_node ${cpp_typesupport_target})
target_link_libraries(herkulex_0602_ex_node ${cpp_typesupport_target})#add...0602_ex

# Link against the serial library manually
target_link_libraries(herkulex_node serial)


# Install
install(TARGETS
  herkulex_node
  herkulex_0602_ex_node #add...0602_ex
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

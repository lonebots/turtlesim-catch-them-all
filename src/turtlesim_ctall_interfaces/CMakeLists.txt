cmake_minimum_required(VERSION 3.8)
project(turtlesim_ctall_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv
"msg/Turtle.msg"
"msg/TurtleArray.msg"
)

set(msg
 "srv/CatchTurtle.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
${srv}
${msg}
)

ament_package()

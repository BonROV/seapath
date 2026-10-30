cmake_minimum_required(VERSION 3.8)

################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
    ament_cmake
    rclcpp
    sensor_msgs
    std_msgs
    nav_msgs
    geometry_msgs
    tf2
    io_interfaces
  )

set(EXTERNAL_DEPENDS

  )

set(EXTERNAL_DEPENDS_LIB

  )


get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")
project(${PACKAGE_NAME})

add_compile_definitions(CMAKE_PACKAGE_NAME=${PACKAGE_NAME})

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###
###  Add dependencies
###

foreach(DEPEND ${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

foreach(DEPEND ${ROS_DEPENDS})
  list(APPEND ROS_DEPENDS_TARGETS ${${DEPEND}_TARGETS})
endforeach(DEPEND)


################
## Find files ##
################

file(GLOB HDRS CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES CONFIGURE_DEPENDS
  "nodes/*.cpp"
)

include_directories(
  include
)

###########
## Build ##
###########

set(EXECUTABLES "")

message("buliding ${PROJECT_NAME} node: ")

foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})
  list(APPEND EXECUTABLES ${NODE_NAME})
  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS})
  target_link_libraries(${NODE_NAME} ${ROS_DEPENDS_TARGETS} ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")
endforeach(NODE_FILE)


message("  Installing: " ${EXECUTABLES})
install(TARGETS
  ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}/
)

#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

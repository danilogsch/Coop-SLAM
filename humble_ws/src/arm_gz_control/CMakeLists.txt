cmake_minimum_required(VERSION 3.8)
project(arm_gz_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# ros package
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# GZ package
find_package(gz-sim7 REQUIRED)
set(IGN_GAZEBO_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-transport12 REQUIRED)
set(IGN_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

find_package(gz-msgs9 REQUIRED)
set(IGN_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

message(STATUS "Compiling against Gazebop Garden")

# include
include_directories(include)

# generate universal_robot_ign lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})

ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    std_msgs
    sensor_msgs
    trajectory_msgs
)

target_link_libraries(${PROJECT_NAME}
  gz-msgs${IGN_MSGS_VER}::core
  gz-transport${IGN_TRANSPORT_VER}::core
)

add_executable(joint_controller src/joint_controller_main.cpp)
target_link_libraries(joint_controller ${PROJECT_NAME})

add_executable(startup_trigger src/timer_teste.cpp)
target_link_libraries(startup_trigger ${PROJECT_NAME})

add_executable(finish_trigger src/timer_teste_fim.cpp)
target_link_libraries(finish_trigger ${PROJECT_NAME})

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install executables
install(TARGETS joint_controller startup_trigger finish_trigger
  DESTINATION lib/${PROJECT_NAME} 
)


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

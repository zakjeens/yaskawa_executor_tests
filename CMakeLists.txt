cmake_minimum_required(VERSION 3.8)
project(yaskawa_executor_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motoros2_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED) 

find_package(action_msgs REQUIRED)

find_package(yaskawa_executor REQUIRED) 


##### Generate the executables #### 


add_executable(testTraj_client src/testTraj_client.cpp)
target_include_directories(testTraj_client PUBLIC include)
ament_target_dependencies(testTraj_client
  ament_cmake
  rclcpp
  rclcpp_action
  sensor_msgs
  motoros2_interfaces
  yaskawa_executor
)

add_executable(dummy_qtp_service src/dummy_qtp_service.cpp)
target_include_directories(dummy_qtp_service PUBLIC include)
ament_target_dependencies(dummy_qtp_service
  ament_cmake
  rclcpp
  rclcpp_action
  sensor_msgs
  motoros2_interfaces
  yaskawa_executor
)
  
##### Install directories and targets #### 
install(
  DIRECTORY src config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
    testTraj_client dummy_qtp_service
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()

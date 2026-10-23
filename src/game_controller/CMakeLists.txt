cmake_minimum_required(VERSION 3.8)
project(game_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(game_controller_interface REQUIRED)

add_executable(game_controller_node src/main.cpp src/game_controller_node.cpp)

ament_target_dependencies(game_controller_node game_controller_interface rclcpp)

target_include_directories(game_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  enable_testing()
  add_executable(game_controller_packet_test test/test_game_controller_packet.cpp)
  target_include_directories(game_controller_packet_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(game_controller_packet_test PRIVATE cxx_std_17)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(game_controller_packet_test PRIVATE -UNDEBUG)
  endif()
  add_test(NAME game_controller_packet_test COMMAND game_controller_packet_test)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(TARGETS
  game_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

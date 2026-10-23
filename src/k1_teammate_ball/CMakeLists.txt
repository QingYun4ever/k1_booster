cmake_minimum_required(VERSION 3.16...3.27)
project(k1_teammate_ball)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 纯 C++ 逻辑库, 不依赖 ROS / brain 类型, 便于独立测试与复用。
add_library(${PROJECT_NAME}
  src/teammate_ball_sharing.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_executable(teammate_ball_sharing_test test/test_teammate_ball_sharing.cpp)
  target_link_libraries(teammate_ball_sharing_test ${PROJECT_NAME})
  target_compile_features(teammate_ball_sharing_test PRIVATE cxx_std_17)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(teammate_ball_sharing_test PRIVATE -UNDEBUG)
  endif()
  add_test(NAME teammate_ball_sharing_test COMMAND teammate_ball_sharing_test)
endif()

ament_package()

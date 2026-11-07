
cmake_minimum_required(VERSION 3.8)
project(uav_sim)

find_package(ament_cmake REQUIRED)

# Install Gazebo worlds, models, launch files, and configuration.
install(
  DIRECTORY
    worlds
    models
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright and cpplint checks until project headers are added.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

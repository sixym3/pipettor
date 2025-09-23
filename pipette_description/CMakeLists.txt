cmake_minimum_required(VERSION 3.16)
project(pipette_description)

find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# Install rviz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Python executables are now in pipette_driver package

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
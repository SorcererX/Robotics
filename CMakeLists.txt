cmake_minimum_required( VERSION 3.16.3 )
project(SepiaCore)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_subdirectory(MovementControl)
add_subdirectory(PS3ControllerControl)
add_subdirectory(GUIControl)

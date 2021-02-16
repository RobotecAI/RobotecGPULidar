cmake_minimum_required(VERSION 3.5)

project(dotnet_cmake_module NONE)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config-version.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})

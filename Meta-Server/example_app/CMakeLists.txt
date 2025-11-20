# (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

cmake_minimum_required(VERSION 3.18.1)

project("haptics_sdk_example")

if(NOT WIN32)
  message(FATAL_ERROR "This CMakeLists.txt only supports building the example for Windows. \
                       To build for Android, please use the Gradle project. \
                       See README.md for more information.")
endif()

set(APP_LIBS_DIR "${PROJECT_SOURCE_DIR}/app/libs/x86-64")
set(APP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/app/libs/include")
set(APP_SRC_DIR "${PROJECT_SOURCE_DIR}/app/src/main/cpp")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_VERBOSE_MAKEFILE ON)

# Enable warnings, but don't warn about our strcpy() use
add_compile_options(/W4)
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

###### OpenGL
find_package(OpenGL)

###### OpenXR
add_definitions(-DXR_USE_PLATFORM_WIN32)
add_library(openxr_loader SHARED IMPORTED)
set_target_properties(
  openxr_loader PROPERTIES
  IMPORTED_LOCATION "${APP_LIBS_DIR}/openxr_loader.dll"
  IMPORTED_IMPLIB "${APP_LIBS_DIR}/openxr_loader.lib"
)

###### Haptics SDK

# [haptics_sdk]
add_library(haptics_sdk SHARED IMPORTED)
set_target_properties(
  haptics_sdk PROPERTIES
  IMPORTED_LOCATION "${APP_LIBS_DIR}/haptics_sdk.dll"
  IMPORTED_IMPLIB "${APP_LIBS_DIR}/haptics_sdk.lib"
)

###### Example App
add_executable(haptics_sdk_example
  "${APP_SRC_DIR}/main_windows.cpp"
  "${APP_SRC_DIR}/program.cpp"
  "${APP_SRC_DIR}/error.cpp"
  "resource.rc"
)
target_include_directories(haptics_sdk_example PRIVATE
  ${APP_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(haptics_sdk_example PRIVATE haptics_sdk)
target_link_libraries(haptics_sdk_example PRIVATE OpenGL::GL)
target_link_libraries(haptics_sdk_example PRIVATE openxr_loader)

add_custom_command(
  TARGET haptics_sdk_example POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          "${APP_LIBS_DIR}/openxr_loader.dll"
          $<TARGET_FILE_DIR:haptics_sdk_example>
  COMMAND ${CMAKE_COMMAND} -E copy
          "${APP_LIBS_DIR}/haptics_sdk.dll"
          $<TARGET_FILE_DIR:haptics_sdk_example>
)

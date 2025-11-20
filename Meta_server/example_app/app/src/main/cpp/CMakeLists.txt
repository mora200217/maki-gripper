# (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

cmake_minimum_required(VERSION 3.18.1)

project("haptics_sdk_example")

if(NOT ANDROID)
  message(FATAL_ERROR "This CMakeLists.txt only supports building the example for Android, \
                       via the Gradle project. To build for Windows, please use the CMakeLists.txt \
                       file in the root directory of the example. \
                       See README.md for more information.")
endif()

set(EXAMPLE_DIR ${PROJECT_SOURCE_DIR}/../../../..)
set(APP_LIBS_DIR ${EXAMPLE_DIR}/app/libs/arm64-v8a)

set(CMAKE_CXX_STANDARD 20)

###### Native App Glue

# See: https://github.com/android-ndk/ndk/issues/381
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -u ANativeActivity_onCreate")


add_library(native_app_glue STATIC
  ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)


###### OpenXR

add_definitions(-DXR_USE_PLATFORM_ANDROID)
add_library(openxr_loader SHARED IMPORTED)
set_property(
  TARGET openxr_loader
  PROPERTY IMPORTED_LOCATION
  ${APP_LIBS_DIR}/libopenxr_loader.so
)


###### Haptics SDK

# [haptics_sdk]
add_library(haptics_sdk SHARED IMPORTED)
set_property(
  TARGET haptics_sdk
  PROPERTY IMPORTED_LOCATION
  ${APP_LIBS_DIR}/libhaptics_sdk.so
)


###### Example App

add_library(haptics_sdk_example SHARED
  main_android.cpp
  program.cpp
  error.cpp
  network_bridge.cpp
)

# Enable warnings, but don't warn about the usage of designated struct initializers
target_compile_options(haptics_sdk_example PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Werror
  -Wno-c99-extensions
  -Wno-missing-field-initializers
)

target_include_directories(haptics_sdk_example PRIVATE
  ${ANDROID_NDK}/sources/android/native_app_glue
  ${EXAMPLE_DIR}/app/libs/include
)

target_link_libraries(haptics_sdk_example
  android
  native_app_glue
  EGL
  GLESv3
  log
  openxr_loader
  haptics_sdk
)

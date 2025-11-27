// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#ifdef __ANDROID__

#include <android/log.h>

#define APP_NAME "haptics_sdk_example"

#define LOG_DEBUG(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, APP_NAME, __VA_ARGS__))
#define LOG_INFO(...) ((void)__android_log_print(ANDROID_LOG_INFO, APP_NAME, __VA_ARGS__))
#define LOG_WARN(...) ((void)__android_log_print(ANDROID_LOG_WARN, APP_NAME, __VA_ARGS__))
#define LOG_ERROR(...) ((void)__android_log_print(ANDROID_LOG_ERROR, APP_NAME, __VA_ARGS__))

#elif _WIN32

#include <iostream>

#define LOG_DEBUG(...)   \
  do {                   \
    printf(__VA_ARGS__); \
    printf("\n");        \
  } while (0)
#define LOG_INFO(...)    \
  do {                   \
    printf(__VA_ARGS__); \
    printf("\n");        \
  } while (0)
#define LOG_WARN(...)             \
  do {                            \
    fprintf(stderr, __VA_ARGS__); \
    fprintf(stderr, "\n");        \
  } while (0)
#define LOG_ERROR(...)            \
  do {                            \
    fprintf(stderr, __VA_ARGS__); \
    fprintf(stderr, "\n");        \
  } while (0)

#endif

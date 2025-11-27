// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#ifdef __ANDROID__
#include <EGL/egl.h>
#endif

#include <openxr/openxr.h>

#include <cstdint>

// Throws an `std::runtime_error` exception, using the last Haptics SDK error message as the
// explanatory string.
void throwHapticsSdkError();

// Checks the Haptics SDK result code, and in case of an error, throws an `std::runtime_error`
// exception, using the last Haptics SDK error message as the explanatory string.
void checkHapticsSdk(int32_t result);

// Checks the OpenXR result code, and in case of an error, throws an `std::runtime_error` that
// mentions `operation` in its explanatory string.
XrResult checkXr(XrResult result, const char* operation);

#ifdef __ANDROID__
// Checks the EGL result code, and in case of an error, throws an std::runtime_error that mentions
// `operation` in its explanatory string.
void checkEgl(EGLBoolean result, const char* operation);
#endif

// Checks `result`, and in case it is false, throws an `std::runtime_error` that
// mentions `error` in its explanatory string.
void check(bool result, const char* error);

#ifdef _WIN32
// Throws an `std::runtime_error` exception, including the error message from `GetLastError()` and
// `operation` in the explanatory string.
void throwWin32Error(const char* operation);

// Checks `result`, and in case it is false, throws an `std::runtime_error` exception, including
// the error message from `GetLastError()` and `operation` in the explanatory string.
void checkWin32(bool result, const char* operation);
#endif

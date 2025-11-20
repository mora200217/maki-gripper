// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "error.hpp"

#include <haptics_sdk/haptics_sdk.h>

#if _WIN32
#include <Windows.h>
#endif

#include <sstream>
#include <stdexcept>

void throwHapticsSdkError() {
  throw std::runtime_error(haptics_sdk_error_message());
}

void checkHapticsSdk(int32_t result) {
  if (HAPTICS_SDK_FAILED(result)) {
    throwHapticsSdkError();
  }
}

XrResult checkXr(XrResult result, const char* operation) {
  if (XR_SUCCEEDED(result)) {
    return result;
  }

  std::ostringstream error;
  error << "OpenXR error: failed to " << operation << " (" << result << ")";
  throw std::runtime_error(error.str());
}

#ifdef __ANDROID__
void checkEgl(EGLBoolean result, const char* operation) {
  if (result == EGL_TRUE) {
    return;
  }

  std::ostringstream error;
  error << "OpenGL ES error: failed to " << operation;
  throw std::runtime_error(error.str());
}
#endif

void check(bool result, const char* error) {
  if (!result) {
    throw std::runtime_error(error);
  }
}

#ifdef _WIN32
void throwWin32Error(const char* operation) {
  const DWORD errorCode = GetLastError();
  LPSTR errorText = nullptr;
  const DWORD errorTextSize = FormatMessageA(
      FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL,
      errorCode,
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      reinterpret_cast<LPSTR>(&errorText),
      0,
      NULL);
  if (errorTextSize == 0 || !errorText) {
    std::ostringstream error;
    error << "Win32 error: failed to " << operation << ": " << errorCode;
    throw std::runtime_error(error.str());
  }

  std::ostringstream error;
  error << "Win32 error: failed to " << operation << ": " << errorText;
  LocalFree(errorText);
  throw std::runtime_error(error.str());
}

void checkWin32(bool result, const char* operation) {
  if (!result) {
    throwWin32Error(operation);
  }
}
#endif

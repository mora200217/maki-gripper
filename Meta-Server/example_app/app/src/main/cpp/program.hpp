// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#ifdef __ANDROID__
#include <android/window.h>
#include <android_native_app_glue.h>

#define XR_USE_GRAPHICS_API_OPENGL_ES
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES/gl.h>
#include <GLES3/gl3.h>

#elif _WIN32

#include <windows.h>

#define XR_USE_GRAPHICS_API_OPENGL
#include <GL/gl.h>
#include <GL/glext.h>
#endif

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <haptics_sdk/haptics_sdk.h>

#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include "haptic_command.h"

struct SwapchainInfo {
  XrSwapchain handle;
  int32_t width;
  int32_t height;
};

#ifdef __ANDROID__
using App = android_app;

#elif _WIN32

struct Win32App {
  HMODULE instance = nullptr;
  HWND window = nullptr;
  HDC deviceContext = nullptr;
};
using App = Win32App;
#endif

class Program {
 public:
  explicit Program(App* app);
  ~Program();

  bool sessionRunning() const;

  void renderFrame();
  void pollOpenXrEvents(bool* exitMainLoop);
  void pollOpenXrActions();

 private:
  void setupOpenXr(App* app);
  void setupGraphics(App* app);
  void setupHaptics();
  void updateHapticClipForIntensity(float intensity);
  void onHapticCommandReceived(const HapticCommand& command);
  void checkForCommands();

  bool isBoolActionClicked(XrAction action, XrPath path) const;
  bool _vibrationActiveLeft = false;
  bool _vibrationActiveRight = false;

  float _currentIntensity = 1.0f;
  std::thread _commandThread;
  bool _stopCommandThread = false;
  ////// Haptics SDK
  int32_t _hapticClip = HAPTICS_SDK_INVALID_ID;
  int32_t _hapticPlayerLeft = HAPTICS_SDK_INVALID_ID;
  int32_t _hapticPlayerRight = HAPTICS_SDK_INVALID_ID;

  ////// OpenXR
  bool _xrSessionRunning = false;
  XrSessionState _xrSessionState = XR_SESSION_STATE_UNKNOWN;
  XrInstance _xrInstance = XR_NULL_HANDLE;
  XrSession _xrSession = XR_NULL_HANDLE;
  XrSystemId _xrSystemId = XR_NULL_SYSTEM_ID;
  XrEnvironmentBlendMode _xrBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
  XrSpace _xrSpace = XR_NULL_HANDLE;
  XrActionSet _xrActionSet = XR_NULL_HANDLE;
  XrAction _xrActionStartHaptic = XR_NULL_HANDLE;
  XrAction _xrActionStopHaptic = XR_NULL_HANDLE;
  XrPath _xrPathLeft = XR_NULL_PATH;
  XrPath _xrPathRight = XR_NULL_PATH;
  std::vector<SwapchainInfo> _xrSwapchains;
#ifdef __ANDROID__
  std::map<XrSwapchain, std::vector<XrSwapchainImageOpenGLESKHR>> _xrSwapchainImages;
#elif _WIN32
  std::map<XrSwapchain, std::vector<XrSwapchainImageOpenGLKHR>> _xrSwapchainImages;
#endif
  std::vector<XrCompositionLayerProjectionView> _xrProjectionViews;

  ////// OpenGL
#ifdef __ANDROID__
  EGLConfig _glConfig = nullptr;
  EGLContext _glContext = EGL_NO_SURFACE;
  EGLDisplay _glDisplay = EGL_NO_DISPLAY;
  EGLSurface _glSurface = EGL_NO_CONTEXT;
#elif _WIN32
  PFNGLGENFRAMEBUFFERSPROC glGenFramebuffers = nullptr;
  PFNGLDELETEFRAMEBUFFERSPROC glDeleteFramebuffers = nullptr;
  PFNGLBINDFRAMEBUFFERPROC glBindFramebuffer = nullptr;
  PFNGLFRAMEBUFFERTEXTURE2DPROC glFramebufferTexture2D = nullptr;
  HGLRC _renderingContext = nullptr;
#endif
  GLuint _glFramebuffer = 0;

  ////// App state
  bool _initialized = false;
};

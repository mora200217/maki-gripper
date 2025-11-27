// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "error.hpp"
#include "logging.hpp"
#include "program.hpp"
#include "resource.h"

#include <thread>

static constexpr const char* exampleAppName = "Haptics SDK Example";

LRESULT APIENTRY WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
  switch (message) {
    case WM_CLOSE: {
      PostQuitMessage(0);
      return 0;
    }
  }
  return DefWindowProcA(hWnd, message, wParam, lParam);
}

Win32App createWindow() {
  LOG_INFO("Starting window setup");
  const HMODULE instance = GetModuleHandleA(NULL);
  checkWin32(instance, "get module handle");

  const HICON icon = LoadIcon(instance, MAKEINTRESOURCE(IDI_APP_ICON));
  checkWin32(icon, "load icon");

  const WNDCLASSA windowClass = {
      .style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC,
      .lpfnWndProc = WndProc,
      .hInstance = instance,
      .hIcon = icon,
      .hCursor = LoadCursor(NULL, IDC_ARROW),
      .lpszClassName = exampleAppName};
  checkWin32(RegisterClassA(&windowClass), "register window class");

  const HWND window = CreateWindowExA(
      WS_EX_APPWINDOW | WS_EX_WINDOWEDGE,
      exampleAppName,
      exampleAppName,
      WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
      0,
      0,
      640,
      480,
      NULL,
      NULL,
      instance,
      NULL);
  checkWin32(window, "create window");

  const HDC deviceContext = GetDC(window);
  const PIXELFORMATDESCRIPTOR pixelFormatDescriptor = {
      .nSize = sizeof(PIXELFORMATDESCRIPTOR),
      .nVersion = 1,
      .dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
      .iPixelType = PFD_TYPE_RGBA,
      .cColorBits = 32,
      .cDepthBits = 24,
      .iLayerType = PFD_MAIN_PLANE,
  };

  const int pixelFormat = ChoosePixelFormat(deviceContext, &pixelFormatDescriptor);
  checkWin32(pixelFormat != 0, "choose pixel format");

  checkWin32(
      SetPixelFormat(deviceContext, pixelFormat, &pixelFormatDescriptor), "set pixel format");

  ShowWindow(window, SW_SHOW);
  SetForegroundWindow(window);
  SetFocus(window);

  LOG_INFO("Finished window setup");
  return Win32App{.instance = instance, .window = window, .deviceContext = deviceContext};
}

void destroyWindow(Win32App app) {
  if (app.window && app.deviceContext) {
    ReleaseDC(app.window, app.deviceContext);
  }

  if (app.window) {
    DestroyWindow(app.window);
  }

  if (app.instance) {
    UnregisterClassA(exampleAppName, app.instance);
  }

  app.window = nullptr;
  app.deviceContext = nullptr;
}

void runEventLoop(Win32App* app) {
  Program program(app);

  while (true) {
    if (!program.sessionRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    MSG message;
    while (PeekMessage(&message, NULL, 0, 0, PM_REMOVE)) {
      if (message.message == WM_QUIT) {
        return;
      }
      TranslateMessage(&message);
      DispatchMessage(&message);
    }

    bool exitLoop = false;
    program.pollOpenXrEvents(&exitLoop);
    if (exitLoop) {
      return;
    }

    if (!program.sessionRunning()) {
      continue;
    }

    program.pollOpenXrActions();
    program.renderFrame();
  }
}

// Use the dedicated GPU instead of the internal GPU. This is required by the Oculus OpenXR runtime.
extern "C" {
__declspec(dllexport) DWORD NvOptimusEnablement = 1;
__declspec(dllexport) DWORD AmdPowerXpressRequestHighPerformance = 1;
}

int main(int, char*[]) {
  try {
    Win32App app = createWindow();
    runEventLoop(&app);
    destroyWindow(app);

    LOG_INFO("Exiting");
  } catch (const std::exception& e) {
    LOG_ERROR("Caught exception: %s", e.what());
  } catch (...) {
    LOG_ERROR("Caught unknown exception");
  }
}

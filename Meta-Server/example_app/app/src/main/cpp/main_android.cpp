// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "logging.hpp"
#include "program.hpp"

#include <android_native_app_glue.h>

bool paused = false;

void appCommandHandler(struct android_app*, int32_t cmd) {
  switch (cmd) {
    case APP_CMD_RESUME: {
      paused = false;
      LOG_INFO("Application resumed");
      break;
    }
    case APP_CMD_PAUSE: {
      paused = true;
      LOG_INFO("Application paused");
      break;
    }
  }
}

void runEventLoop(android_app* app) {
  Program program(app);
  app->onAppCmd = appCommandHandler;

  bool exitLoop = false;
  while (!app->destroyRequested) {
    int eventCount = 0;
    android_poll_source* source = nullptr;

    const int timeout = !paused || program.sessionRunning() ? 0 : -1;
    if (ALooper_pollAll(timeout, nullptr, &eventCount, reinterpret_cast<void**>(&source)) ==
        ALOOPER_POLL_ERROR) {
      throw std::runtime_error("an error occurred while polling application events");
    }

    if (source != nullptr) {
      source->process(app, source);
    }

    program.pollOpenXrEvents(&exitLoop);

    if (exitLoop) {
      ANativeActivity_finish(app->activity);
      break;
    }

    if (!program.sessionRunning()) {
      continue;
    }

    program.pollOpenXrActions();
    program.renderFrame();
  }
}

void android_main(android_app* app) {
  try {
    JNIEnv* jniEnv = nullptr;
    app->activity->vm->AttachCurrentThread(&jniEnv, nullptr);

    runEventLoop(app);

    LOG_INFO("Exiting");
    app->activity->vm->DetachCurrentThread();
  } catch (const std::exception& e) {
    LOG_ERROR("Caught exception: %s", e.what());
  } catch (...) {
    LOG_ERROR("Caught unknown exception");
  }
}

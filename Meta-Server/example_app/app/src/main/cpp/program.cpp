// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "program.hpp"

#include "error.hpp"
#include "logging.hpp"
#include "network_bridge.h"

void Program::setupOpenXr(App* app) {
  LOG_INFO("Starting OpenXR setup");

#ifdef __ANDROID__
  ////// Initialize the loader
  {
    PFN_xrInitializeLoaderKHR initializeLoader = nullptr;
    checkXr(
        xrGetInstanceProcAddr(
            XR_NULL_HANDLE,
            "xrInitializeLoaderKHR",
            reinterpret_cast<PFN_xrVoidFunction*>(&initializeLoader)),
        "initialize loader");

    const XrLoaderInitInfoAndroidKHR info = {
        .type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR,
        .applicationVM = app->activity->vm,
        .applicationContext = app->activity->clazz,
    };
    initializeLoader(reinterpret_cast<const XrLoaderInitInfoBaseHeaderKHR*>(&info));
  }
#endif

  ////// Create the instance
  {
    std::vector<const char*> extensions = {
#ifdef __ANDROID__
        XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME, XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME
#elif _WIN32
        XR_KHR_OPENGL_ENABLE_EXTENSION_NAME
#endif
    };

    //// [haptics_sdk] Get the OpenXR extensions required by the Haptics SDK
    int extensionCount;
    checkHapticsSdk(haptics_sdk_get_openxr_extension_count(&extensionCount));
    for (int i = 0; i < extensionCount; i++) {
      const char* extension = haptics_sdk_get_openxr_extension(i);
      if (extension == nullptr) {
        throwHapticsSdkError();
      }
      extensions.push_back(extension);
    }
    //// [haptics_sdk]

#ifdef __ANDROID__
    const XrInstanceCreateInfoAndroidKHR androidInstanceCreateInfo = {
        .type = XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR,
        .applicationVM = app->activity->vm,
        .applicationActivity = app->activity->clazz,
    };
    XrInstanceCreateInfo instanceCreateInfo{
        .type = XR_TYPE_INSTANCE_CREATE_INFO,
        .next = &androidInstanceCreateInfo,
        .enabledExtensionCount = static_cast<uint32_t>(extensions.size()),
        .enabledExtensionNames = extensions.data(),
    };
#elif _WIN32
    XrInstanceCreateInfo instanceCreateInfo{
        .type = XR_TYPE_INSTANCE_CREATE_INFO,
        .next = nullptr,
        .enabledExtensionCount = static_cast<uint32_t>(extensions.size()),
        .enabledExtensionNames = extensions.data(),
    };
#endif

    strcpy(instanceCreateInfo.applicationInfo.applicationName, "Haptics SDK Example");
    instanceCreateInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

    checkXr(xrCreateInstance(&instanceCreateInfo, &_xrInstance), "create instance");

    //// [haptics_sdk] Pass the OpenXR instance to the Haptics SDK
    checkHapticsSdk(haptics_sdk_set_openxr_instance(_xrInstance));
    //// [haptics_sdk]
  }

  ////// Get the system ID
  {
    const XrSystemGetInfo info = {
        .type = XR_TYPE_SYSTEM_GET_INFO, .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
    checkXr(xrGetSystem(_xrInstance, &info, &_xrSystemId), "get system ID");
  }

  ////// Get the default environment blend mode
  {
    uint32_t count = 0;
    const XrViewConfigurationType viewType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    checkXr(
        xrEnumerateEnvironmentBlendModes(_xrInstance, _xrSystemId, viewType, 0, &count, nullptr),
        "get blend mode count");
    check(count > 0, "no available blend modes");
    std::vector<XrEnvironmentBlendMode> blendModes(count);
    checkXr(
        xrEnumerateEnvironmentBlendModes(
            _xrInstance, _xrSystemId, viewType, count, &count, blendModes.data()),
        "get blend modes");
    _xrBlendMode = blendModes.front();
  }

  ////// Setup OpenGL ES
  this->setupGraphics(app);

  ////// Create the session
  {
#ifdef __ANDROID__
    const XrGraphicsBindingOpenGLESAndroidKHR graphicsBinding = {
        .type = XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR,
        .display = _glDisplay,
        .config = _glConfig,
        .context = _glContext,
    };
#elif _WIN32
    XrGraphicsBindingOpenGLWin32KHR graphicsBinding = {
        .type = XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR,
        .hDC = app->deviceContext,
        .hGLRC = _renderingContext,
    };
#endif
    const XrSessionCreateInfo info = {
        .type = XR_TYPE_SESSION_CREATE_INFO,
        .next = &graphicsBinding,
        .systemId = _xrSystemId,
    };
    LOG_INFO("creating session");
    checkXr(xrCreateSession(_xrInstance, &info, &_xrSession), "create session");

    //// [haptics_sdk] Pass the OpenXR session to the Haptics SDK
    checkHapticsSdk(haptics_sdk_set_openxr_session(_xrSession));
    //// [haptics_sdk]
  }

  ////// Create the reference space
  {
    const XrReferenceSpaceCreateInfo info = {
        .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
        .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE,
        .poseInReferenceSpace = XrPosef{.orientation = XrQuaternionf{.w = 1}}};
    checkXr(xrCreateReferenceSpace(_xrSession, &info, &_xrSpace), "create reference space");
  }

  ////// Create the action set and actions
  {
    XrActionSetCreateInfo info = {.type = XR_TYPE_ACTION_SET_CREATE_INFO};
    strcpy(info.actionSetName, "haptics_sdk_example");
    strcpy(info.localizedActionSetName, "Haptics SDK Example");
    checkXr(xrCreateActionSet(_xrInstance, &info, &_xrActionSet), "create action set");

    //// [haptics_sdk] Pass the OpenXR action set to the Haptics SDK
    checkHapticsSdk(haptics_sdk_set_openxr_action_set(_xrActionSet));
    //// [haptics_sdk]
  }

  ////// Create actions to receive controller input
  {
    checkXr(xrStringToPath(_xrInstance, "/user/hand/left", &_xrPathLeft), "get path for left hand");
    checkXr(
        xrStringToPath(_xrInstance, "/user/hand/right", &_xrPathRight), "get path for right hand");

    const XrPath bothHands[] = {_xrPathLeft, _xrPathRight};

    XrActionCreateInfo info{
        .type = XR_TYPE_ACTION_CREATE_INFO,
        .actionType = XR_ACTION_TYPE_BOOLEAN_INPUT,
        .countSubactionPaths = 2,
        .subactionPaths = bothHands,
    };

    strcpy(info.actionName, "start_haptic");
    strcpy(info.localizedActionName, "Start Haptic");
    checkXr(xrCreateAction(_xrActionSet, &info, &_xrActionStartHaptic), "create start action");

    strcpy(info.actionName, "stop_haptic");
    strcpy(info.localizedActionName, "Stop Haptic");
    checkXr(xrCreateAction(_xrActionSet, &info, &_xrActionStopHaptic), "create stop action");
  }

  ////// Suggest interaction profile bindings
  {
    XrPath pathTriggerLeft, pathTriggerRight, pathClickA, pathClickX;
    checkXr(
        xrStringToPath(_xrInstance, "/user/hand/left/input/trigger/value", &pathTriggerLeft),
        "get path for left trigger");
    checkXr(
        xrStringToPath(_xrInstance, "/user/hand/right/input/trigger/value", &pathTriggerRight),
        "get path for right trigger");
    checkXr(
        xrStringToPath(_xrInstance, "/user/hand/left/input/x/click", &pathClickX),
        "get path for X button");
    checkXr(
        xrStringToPath(_xrInstance, "/user/hand/right/input/a/click", &pathClickA),
        "get path for A button");

    std::vector<XrActionSuggestedBinding> bindings{{
        {_xrActionStartHaptic, pathTriggerLeft},
        {_xrActionStartHaptic, pathTriggerRight},
        {_xrActionStopHaptic, pathClickX},
        {_xrActionStopHaptic, pathClickA},
    }};

    //// [haptics_sdk] Include the Haptic SDK's bindings when calling
    ////               xrSuggestInteractionProfileBindings().
    int bindingCount;
    checkHapticsSdk(haptics_sdk_get_openxr_suggested_binding_count(&bindingCount));
    for (int i = 0; i < bindingCount; i++) {
      const XrActionSuggestedBinding binding = haptics_sdk_get_openxr_suggested_binding(i);
      if (binding.binding == XR_NULL_PATH || binding.action == XR_NULL_HANDLE) {
        throwHapticsSdkError();
      }
      bindings.push_back(binding);
    }
    //// [haptics_sdk]

    XrPath interactionProfilePath;
    checkXr(
        xrStringToPath(
            _xrInstance, "/interaction_profiles/oculus/touch_controller", &interactionProfilePath),
        "get interaction profile path");
    XrInteractionProfileSuggestedBinding suggestedBindings = {
        .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
        .interactionProfile = interactionProfilePath,
        .countSuggestedBindings = static_cast<uint32_t>(bindings.size()),
        .suggestedBindings = bindings.data(),
    };
    checkXr(
        xrSuggestInteractionProfileBindings(_xrInstance, &suggestedBindings),
        "suggest interaction profile bindings");
  }

  ////// Attach the action set to the session
  {
    const XrSessionActionSetsAttachInfo info{
        .type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO,
        .countActionSets = 1,
        .actionSets = &_xrActionSet,
    };
    checkXr(xrAttachSessionActionSets(_xrSession, &info), "attach session action set");
  }

  ////// Create the view swapchains
  {
    uint32_t viewCount = 0;
    checkXr(
        xrEnumerateViewConfigurationViews(
            _xrInstance,
            _xrSystemId,
            XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
            0,
            &viewCount,
            nullptr),
        "get view count");
    check(viewCount > 0, "view count is zero");

    std::vector<XrViewConfigurationView> viewConfigs;
    viewConfigs.resize(viewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
    checkXr(
        xrEnumerateViewConfigurationViews(
            _xrInstance,
            _xrSystemId,
            XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
            viewCount,
            &viewCount,
            viewConfigs.data()),
        "get view configs");

    uint32_t swapchainCount = 0;
    checkXr(
        xrEnumerateSwapchainFormats(_xrSession, 0, &swapchainCount, nullptr),
        "get swapchain format count");
    check(swapchainCount > 0, "swapchain format count is zero");
    std::vector<int64_t> swapchainFormats(swapchainCount);
    checkXr(
        xrEnumerateSwapchainFormats(
            _xrSession, swapchainCount, &swapchainCount, swapchainFormats.data()),
        "get swapchain formats");

    const auto swapchainFormat =
        std::find_if(swapchainFormats.begin(), swapchainFormats.end(), [](int64_t format) {
          return format == GL_RGBA8 || format == GL_RGBA8_SNORM || format == GL_SRGB8_ALPHA8;
        });
    check(swapchainFormat != swapchainFormats.end(), "no appropriate swapchain format found");

    for (const XrViewConfigurationView& config : viewConfigs) {
      const XrSwapchainCreateInfo info = {
          .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
          .format = *swapchainFormat,
          .sampleCount = 1,
          .width = config.recommendedImageRectWidth,
          .height = config.recommendedImageRectHeight,
          .faceCount = 1,
          .arraySize = 1,
          .mipCount = 1,
      };
      XrSwapchain swapchain = XR_NULL_HANDLE;
      checkXr(xrCreateSwapchain(_xrSession, &info, &swapchain), "create swapchain");

      uint32_t imageCount = 0;
      checkXr(
          xrEnumerateSwapchainImages(swapchain, 0, &imageCount, nullptr),
          "get swapchain image count");
#ifdef __ANDROID__
      std::vector<XrSwapchainImageOpenGLESKHR> swapchainImages(
          imageCount, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR});
#elif _WIN32
      std::vector<XrSwapchainImageOpenGLKHR> swapchainImages(
          imageCount, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR});
#endif
      checkXr(
          xrEnumerateSwapchainImages(
              swapchain,
              imageCount,
              &imageCount,
              reinterpret_cast<XrSwapchainImageBaseHeader*>(swapchainImages.data())),
          "get swapchain images");

      _xrSwapchains.push_back(SwapchainInfo{
          .handle = swapchain,
          .width = static_cast<int32_t>(config.recommendedImageRectWidth),
          .height = static_cast<int32_t>(config.recommendedImageRectHeight),
      });
      _xrSwapchainImages[swapchain] = std::move(swapchainImages);
    }

    _xrProjectionViews.resize(_xrSwapchains.size());
  }

  LOG_INFO("Finished OpenXR setup");
}

void Program::setupGraphics([[maybe_unused]] App* app) {
  LOG_INFO("Starting graphics setup");
#ifdef __ANDROID__
  ////// Initialize OpenGL ES
  _glDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  checkEgl(eglInitialize(_glDisplay, nullptr, nullptr), "initialize OpenGL ES");
  checkEgl(eglBindAPI(EGL_OPENGL_ES_API), "bind ES API");

  ////// Find an appropriate EGL config
  {
    EGLint configCount = 0;
    checkEgl(eglGetConfigs(_glDisplay, nullptr, 0, &configCount), "get number of configs");
    check(configCount > 0, "missing configs");
    std::vector<EGLConfig> configs(configCount);
    checkEgl(
        eglGetConfigs(
            _glDisplay, configs.data(), static_cast<EGLint>(configs.size()), &configCount),
        "get configs");

    for (const EGLConfig& config : configs) {
      EGLint red, green, blue, depth;
      if (eglGetConfigAttrib(_glDisplay, config, EGL_RED_SIZE, &red) &&
          eglGetConfigAttrib(_glDisplay, config, EGL_GREEN_SIZE, &green) &&
          eglGetConfigAttrib(_glDisplay, config, EGL_BLUE_SIZE, &blue) &&
          eglGetConfigAttrib(_glDisplay, config, EGL_DEPTH_SIZE, &depth)) {
        if (red == 8 && green == 8 && blue == 8 && depth == 0) {
          _glConfig = config;
          break;
        }
      }
    }

    check(_glConfig, "failed to find compatible EGL config");
  }

  ////// Create the EGL context, surface, and framebuffers
  {
    const EGLint attribs[] = {EGL_CONTEXT_MAJOR_VERSION, 3, EGL_NONE};
    _glContext = eglCreateContext(_glDisplay, _glConfig, EGL_NO_CONTEXT, attribs);
    check(_glContext, "failed to create GL context");

    ////// Create the EGL surface
    const EGLint surfaceAttribs[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    _glSurface = eglCreatePbufferSurface(_glDisplay, _glConfig, surfaceAttribs);
    check(_glSurface != EGL_NO_SURFACE, "failed to create EGL surface");

    ////// Bind the EGL context
    checkEgl(eglMakeCurrent(_glDisplay, _glSurface, _glSurface, _glContext), "make EGL current");
  }
#elif _WIN32
  _renderingContext = wglCreateContext(app->deviceContext);
  checkWin32(_renderingContext, "create OpenGL context");
  checkWin32(wglMakeCurrent(app->deviceContext, _renderingContext), "make OpenGL context current");

  glGenFramebuffers =
      reinterpret_cast<PFNGLGENFRAMEBUFFERSPROC>(wglGetProcAddress("glGenFramebuffers"));
  glDeleteFramebuffers =
      reinterpret_cast<PFNGLDELETEFRAMEBUFFERSPROC>(wglGetProcAddress("glDeleteFramebuffers"));
  glBindFramebuffer =
      reinterpret_cast<PFNGLBINDFRAMEBUFFERPROC>(wglGetProcAddress("glBindFramebuffer"));
  glFramebufferTexture2D =
      reinterpret_cast<PFNGLFRAMEBUFFERTEXTURE2DPROC>(wglGetProcAddress("glFramebufferTexture2D"));
  checkWin32(
      glGenFramebuffers && glDeleteFramebuffers && glBindFramebuffer && glFramebufferTexture2D,
      "get extensions");
#endif

  ////// Set up a framebuffer for rendering into OpenXR images
  glGenFramebuffers(1, &_glFramebuffer);
  check(_glFramebuffer != 0, "failed to create frame buffer");

  ////// Check that OpenXR requirements are met
  {
#ifdef __ANDROID__
    PFN_xrGetOpenGLESGraphicsRequirementsKHR getRequirements = nullptr;
    checkXr(
        xrGetInstanceProcAddr(
            _xrInstance,
            "xrGetOpenGLESGraphicsRequirementsKHR",
            reinterpret_cast<PFN_xrVoidFunction*>(&getRequirements)),
        "get graphics requirements function");
    XrGraphicsRequirementsOpenGLESKHR requirements{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
#elif _WIN32
    PFN_xrGetOpenGLGraphicsRequirementsKHR getRequirements = nullptr;
    checkXr(
        xrGetInstanceProcAddr(
            _xrInstance,
            "xrGetOpenGLGraphicsRequirementsKHR",
            reinterpret_cast<PFN_xrVoidFunction*>(&getRequirements)),
        "get graphics requirements function");
    XrGraphicsRequirementsOpenGLKHR requirements{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR};
#endif
    checkXr(getRequirements(_xrInstance, _xrSystemId, &requirements), "get graphics requirements");

    GLint glMajor = 0;
    GLint glMinor = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &glMajor);
    glGetIntegerv(GL_MINOR_VERSION, &glMinor);
    const XrVersion glVersion = XR_MAKE_VERSION(glMajor, glMinor, 0);
    check(glVersion >= requirements.minApiVersionSupported, "insufficient OpenGL version");
  }
  LOG_INFO("Finished graphics setup");
}

void Program::setupHaptics() {
  // The contents of a .haptic file, typically this would be loaded via the asset manager.
  const char hapticClipData[] = R"(
{
  "version": {
    "major": 1,
    "minor": 0,
    "patch": 0
  },
  "signals": {
    "continuous": {
      "envelopes": {
        "amplitude": [
          {
            "time": 0.0,
            "amplitude": 0.0
          },
          {
            "time": 2.0,
            "amplitude": 1.0
          },
          {
            "time": 4.0,
            "amplitude": 0.0
          }
        ],
        "frequency": [
          {
            "time": 0.0,
            "frequency": 0.0
          },
          {
            "time": 1.5,
            "frequency": 1.0
          },
          {
            "time": 3.0,
            "frequency": 0.0
          },
          {
            "time": 4.0,
            "frequency": 1.0
          }
        ]
      }
    }
  }
}
)";

  //// [haptics_sdk] Load a clip and create players
  checkHapticsSdk(haptics_sdk_load_clip(hapticClipData, sizeof(hapticClipData) - 1, &_hapticClip));
  checkHapticsSdk(haptics_sdk_create_player(&_hapticPlayerLeft));
  checkHapticsSdk(haptics_sdk_create_player(&_hapticPlayerRight));
  checkHapticsSdk(haptics_sdk_player_set_clip(_hapticPlayerLeft, _hapticClip));
  checkHapticsSdk(haptics_sdk_player_set_clip(_hapticPlayerRight, _hapticClip));
  //// [haptics_sdk]
}

void Program::renderFrame() {
  XrFrameState frameState{.type = XR_TYPE_FRAME_STATE};

  ////// Begin the frame
  {
    const XrFrameWaitInfo frameWaitInfo{.type = XR_TYPE_FRAME_WAIT_INFO};
    checkXr(xrWaitFrame(_xrSession, &frameWaitInfo, &frameState), "wait for frame");

    const XrFrameBeginInfo frameBeginInfo{.type = XR_TYPE_FRAME_BEGIN_INFO};
    checkXr(xrBeginFrame(_xrSession, &frameBeginInfo), "begin frame");
  }

  XrFrameEndInfo frameEndInfo = {
      .type = XR_TYPE_FRAME_END_INFO,
      .displayTime = frameState.predictedDisplayTime,
      .environmentBlendMode = _xrBlendMode,
  };

  ////// Exit if no rendering is required
  if (frameState.shouldRender == XR_FALSE) {
    checkXr(xrEndFrame(_xrSession, &frameEndInfo), "end frame");
    return;
  }

  std::vector<XrView> xrViews;
  xrViews.resize(_xrSwapchains.size(), {.type = XR_TYPE_VIEW});

  ////// Update the views
  {
    XrViewState viewState = {.type = XR_TYPE_VIEW_STATE};
    uint32_t viewCountOutput = 0;

    const XrViewLocateInfo viewLocateInfo{
        .type = XR_TYPE_VIEW_LOCATE_INFO,
        .viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
        .displayTime = frameState.predictedDisplayTime,
        .space = _xrSpace,
    };

    checkXr(
        xrLocateViews(
            _xrSession,
            &viewLocateInfo,
            &viewState,
            static_cast<int32_t>(xrViews.size()),
            &viewCountOutput,
            xrViews.data()),
        "locate views");
    check(
        viewCountOutput == _xrSwapchains.size(), "invalid view count returned from xrLocateViews");
  }

  /////// Render into each swapchain's image
  for (size_t i = 0; i < _xrSwapchains.size(); i++) {
    const SwapchainInfo& swapchain = _xrSwapchains[i];
    uint32_t swapchainImageIndex = 0;

    ////// Acquire the swapchain image
    {
      const XrSwapchainImageAcquireInfo acquireInfo{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
      checkXr(
          xrAcquireSwapchainImage(swapchain.handle, &acquireInfo, &swapchainImageIndex),
          "acquire swapchain image");

      const XrSwapchainImageWaitInfo waitInfo = {
          .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO,
          .timeout = XR_INFINITE_DURATION,
      };
      checkXr(xrWaitSwapchainImage(swapchain.handle, &waitInfo), "wait for swapchain image");
    }

    ////// Clear the image
    {
      const uint32_t image = _xrSwapchainImages[swapchain.handle][swapchainImageIndex].image;

      glBindFramebuffer(GL_FRAMEBUFFER, _glFramebuffer);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, image, 0);

      glClearColor(0.3f, 0.3f, 0.6f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);

      glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    ////// Release the image
    {
      XrCompositionLayerProjectionView& projectionView = _xrProjectionViews[i];
      projectionView.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
      projectionView.pose = xrViews[i].pose;
      projectionView.fov = xrViews[i].fov;
      projectionView.subImage.swapchain = swapchain.handle;
      projectionView.subImage.imageRect.extent = {swapchain.width, swapchain.height};

      const XrSwapchainImageReleaseInfo releaseInfo{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
      checkXr(xrReleaseSwapchainImage(swapchain.handle, &releaseInfo), "release swapchain image");
    }
  }

  ////// End the frame
  {
    const XrCompositionLayerProjection layer = {
        .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION,
        .space = _xrSpace,
        .viewCount = static_cast<uint32_t>(_xrProjectionViews.size()),
        .views = _xrProjectionViews.data(),
    };
    const XrCompositionLayerBaseHeader* const layers[] = {
        reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer),
    };

    frameEndInfo.layerCount = 1;
    frameEndInfo.layers = reinterpret_cast<const XrCompositionLayerBaseHeader* const*>(&layers);
    checkXr(xrEndFrame(_xrSession, &frameEndInfo), "end frame");
  }
}

void Program::pollOpenXrEvents(bool* exitMainLoop) {
  if (!_xrInstance) {
    return;
  }

  for (;;) {
    XrEventDataBuffer eventDataBuffer = {.type = XR_TYPE_EVENT_DATA_BUFFER};
    if (xrPollEvent(_xrInstance, &eventDataBuffer) != XR_SUCCESS) {
      break;
    }

    switch (eventDataBuffer.type) {
      case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
        const XrEventDataSessionStateChanged* stateEvent =
            reinterpret_cast<const XrEventDataSessionStateChanged*>(&eventDataBuffer);
        _xrSessionState = stateEvent->state;
        LOG_INFO("OpenXR session state changed: %i", _xrSessionState);

        //// [haptics_sdk] Pass the new session state to the Haptics SDK
        checkHapticsSdk(haptics_sdk_set_openxr_session_state(_xrSessionState));
        //// [haptics_sdk]

        switch (_xrSessionState) {
          case XR_SESSION_STATE_READY: {
            const XrSessionBeginInfo info = {
                .type = XR_TYPE_SESSION_BEGIN_INFO,
                .primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
            };
            checkXr(xrBeginSession(_xrSession, &info), "begin session");
            _xrSessionRunning = true;
            break;
          }
          case XR_SESSION_STATE_STOPPING: {
            _xrSessionRunning = false;
            checkXr(xrEndSession(_xrSession), "end session");
            break;
          }
          case XR_SESSION_STATE_EXITING:
          case XR_SESSION_STATE_LOSS_PENDING: {
            *exitMainLoop = true;
            break;
          }
          default:
            break;
        }
        break;
      }
      default: {
        LOG_INFO("OpenXR event: ignored (%d)", eventDataBuffer.type);
        break;
      }
    }
  }
}

void Program::pollOpenXrActions() {
  ////// Sync actions
  {
    const XrActiveActionSet actionSet{_xrActionSet, XR_NULL_PATH};
    XrActionsSyncInfo actionSyncInfo{
        .type = XR_TYPE_ACTIONS_SYNC_INFO,
        .countActiveActionSets = 1,
        .activeActionSets = &actionSet,
    };

    checkXr(xrSyncActions(_xrSession, &actionSyncInfo), "sync actions");
  }

  //// [haptics_sdk] Respond to actions and trigger haptics
  if (this->isBoolActionClicked(_xrActionStartHaptic, _xrPathLeft)) {
      LOG_INFO("Gatillo izquierdo presionado - Activando haptics izquierdo");
      checkHapticsSdk(haptics_sdk_player_play(_hapticPlayerLeft, HAPTICS_SDK_CONTROLLER_LEFT));
      NetworkBridge::sendHapticEvent("activated", "left", 1.0f);
  }

  if (this->isBoolActionClicked(_xrActionStartHaptic, _xrPathRight)) {
      LOG_INFO("Gatillo derecho presionado - Activando haptics derecho");
      checkHapticsSdk(haptics_sdk_player_play(_hapticPlayerRight, HAPTICS_SDK_CONTROLLER_RIGHT));
      NetworkBridge::sendHapticEvent("activated", "right", 1.0f);
  }

  if (this->isBoolActionClicked(_xrActionStopHaptic, _xrPathLeft)) {
      LOG_INFO("Botón X presionado - Desactivando haptics izquierdo");
      checkHapticsSdk(haptics_sdk_player_stop(_hapticPlayerLeft));
      NetworkBridge::sendHapticEvent("activated", "right", 1.0f);
  }

  if (this->isBoolActionClicked(_xrActionStopHaptic, _xrPathRight)) {
      LOG_INFO("Botón A presionado - Desactivando haptics derecho");
      checkHapticsSdk(haptics_sdk_player_stop(_hapticPlayerRight));
      NetworkBridge::sendHapticEvent("activated", "right", 1.0f);
  }
  //// [haptics_sdk]
}

bool Program::isBoolActionClicked(XrAction action, XrPath path) const {
  const XrActionStateGetInfo info = {
      .type = XR_TYPE_ACTION_STATE_GET_INFO, .action = action, .subactionPath = path};
  XrActionStateBoolean state = {.type = XR_TYPE_ACTION_STATE_BOOLEAN};
  checkXr(xrGetActionStateBoolean(_xrSession, &info, &state), "get boolean action state");
  return state.changedSinceLastSync && state.isActive && state.currentState;
}

Program::Program(App* app) {
  if (!_initialized) {
    LOG_INFO("Starting program initialization");
    LOG_INFO("Initializing NetworkBridge...");
    NetworkBridge::initialize(app->activity->vm, app->activity->clazz);
    this->setupOpenXr(app);
    this->setupHaptics();
    _initialized = true;
    LOG_INFO("Finished program initialization");
  }
}

Program::~Program() {
  LOG_INFO("Starting program uninitialization");
  _initialized = false;

  //// [haptics_sdk] Free resources and uninitialize the SDK
  if (_hapticClip != HAPTICS_SDK_INVALID_ID) {
    haptics_sdk_release_clip(_hapticClip);
    _hapticClip = HAPTICS_SDK_INVALID_ID;
  }

  if (_hapticPlayerLeft != HAPTICS_SDK_INVALID_ID) {
    haptics_sdk_release_player(_hapticPlayerLeft);
    _hapticPlayerLeft = HAPTICS_SDK_INVALID_ID;
  }

  if (_hapticPlayerRight != HAPTICS_SDK_INVALID_ID) {
    haptics_sdk_release_player(_hapticPlayerRight);
    _hapticPlayerRight = HAPTICS_SDK_INVALID_ID;
  }

  bool hapticsInitialized = false;
  haptics_sdk_initialized(&hapticsInitialized);
  if (hapticsInitialized) {
    haptics_sdk_uninitialize();
  }
  //// [haptics_sdk]

  if (_xrActionSet != XR_NULL_HANDLE) {
    xrDestroyActionSet(_xrActionSet);
    _xrActionSet = XR_NULL_HANDLE;
  }

  _xrProjectionViews.clear();
  _xrSwapchainImages.clear();

  for (SwapchainInfo swapchain : _xrSwapchains) {
    xrDestroySwapchain(swapchain.handle);
  }
  _xrSwapchains.clear();

  if (_xrSpace != XR_NULL_HANDLE) {
    xrDestroySpace(_xrSpace);
    _xrSpace = XR_NULL_HANDLE;
  }

  if (_xrSession != XR_NULL_HANDLE) {
    xrDestroySession(_xrSession);
    _xrSession = XR_NULL_HANDLE;
  }

  if (_xrInstance != XR_NULL_HANDLE) {
    xrDestroyInstance(_xrInstance);
    _xrInstance = XR_NULL_HANDLE;
  }

  if (_glFramebuffer != 0) {
    glDeleteFramebuffers(1, &_glFramebuffer);
    _glFramebuffer = 0;
  }

#ifdef __ANDROID__
  if (_glDisplay != EGL_NO_DISPLAY) {
    eglMakeCurrent(_glDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    if (_glContext != EGL_NO_CONTEXT) {
      eglDestroyContext(_glDisplay, _glContext);
      _glContext = EGL_NO_CONTEXT;
    }

    if (_glSurface != EGL_NO_SURFACE) {
      eglDestroySurface(_glDisplay, _glSurface);
      _glSurface = EGL_NO_CONTEXT;
    }

    eglTerminate(_glDisplay);
    _glDisplay = EGL_NO_DISPLAY;
  }
#elif _WIN32
  wglMakeCurrent(nullptr, nullptr);

  if (_renderingContext) {
    wglDeleteContext(_renderingContext);
    _renderingContext = nullptr;
  }
#endif
  LOG_INFO("Finished program uninitialization");
}

bool Program::sessionRunning() const {
  return _xrSessionRunning;
}

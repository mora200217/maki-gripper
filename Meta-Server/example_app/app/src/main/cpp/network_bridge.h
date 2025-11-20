//
// Created by jaime on 20/11/2025.
//

#ifndef HAPTICS_SDK_OPENXR_EXAMPLE_NETWORK_BRIDGE_H
#define HAPTICS_SDK_OPENXR_EXAMPLE_NETWORK_BRIDGE_H

#include <jni.h>
#include <string>

class NetworkBridge {
public:
    static void initialize(JavaVM* vm, jobject context);
    static void sendHapticEvent(const std::string& eventType,
                                const std::string& controller,
                                float intensity);
    static void setServerUrl(const std::string& url);

private:
    static JavaVM* g_vm;
    static jobject g_context;
    static jclass g_jniBridgeClass;
    static jmethodID g_onHapticEventMethod;
    static std::string g_serverUrl;

    static void callJavaMethod(const std::string& eventType,
                               const std::string& controller,
                               float intensity);
};

#endif //HAPTICS_SDK_OPENXR_EXAMPLE_NETWORK_BRIDGE_H

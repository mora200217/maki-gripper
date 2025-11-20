// network_bridge.cpp (corregido final)
#include "network_bridge.h"
#include <android/log.h>
#include <jni.h>

#define LOG_TAG "NetworkBridge"
#define LOG_INFO(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOG_ERROR(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

JavaVM* NetworkBridge::g_vm = nullptr;
jobject NetworkBridge::g_context = nullptr;
jclass NetworkBridge::g_jniBridgeClass = nullptr;
jmethodID NetworkBridge::g_onHapticEventMethod = nullptr;
std::string NetworkBridge::g_serverUrl = "https://yourserver.com/api/haptics";

static bool attachIfNeeded(JavaVM* vm, JNIEnv** outEnv, bool* attachedHere) {
    if (!vm || !outEnv || !attachedHere) return false;
    *outEnv = nullptr;
    // GetEnv expects a void** for the environment pointer
    jint getEnvResult = vm->GetEnv(reinterpret_cast<void**>(outEnv), JNI_VERSION_1_6);
    if (getEnvResult == JNI_OK) {
        *attachedHere = false;
        return true;
    } else if (getEnvResult == JNI_EDETACHED) {
        // AttachCurrentThread expects JNIEnv** (la dirección de la variable JNIEnv*)
        if (vm->AttachCurrentThread(outEnv, nullptr) != JNI_OK) {
            __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "Failed to AttachCurrentThread");
            return false;
        }
        *attachedHere = true;
        return true;
    } else {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "GetEnv returned unexpected value: %d", getEnvResult);
        return false;
    }
}

void NetworkBridge::initialize(JavaVM* vm, jobject context) {
    if (!vm || !context) {
        LOG_ERROR("initialize called with null vm or context");
        return;
    }

    g_vm = vm;

    JNIEnv* env = nullptr;
    bool attachedHere = false;
    if (!attachIfNeeded(g_vm, &env, &attachedHere)) {
        LOG_ERROR("Failed to get JNIEnv in initialize");
        return;
    }

    // Guardar contexto como global ref
    g_context = env->NewGlobalRef(context);

    // Obtener ClassLoader del contexto (actividad) para evitar FindClass desde hilos nativos
    jclass activityClass = env->GetObjectClass(context);
    if (!activityClass) {
        LOG_ERROR("Failed to get activity class");
        if (attachedHere) g_vm->DetachCurrentThread();
        return;
    }

    jmethodID getClassLoader = env->GetMethodID(activityClass, "getClassLoader", "()Ljava/lang/ClassLoader;");
    if (!getClassLoader) {
        LOG_ERROR("Failed to get getClassLoader method");
        env->DeleteLocalRef(activityClass);
        if (attachedHere) g_vm->DetachCurrentThread();
        return;
    }

    jobject classLoader = env->CallObjectMethod(context, getClassLoader);
    if (env->ExceptionCheck() || !classLoader) {
        env->ExceptionDescribe();
        env->ExceptionClear();
        LOG_ERROR("Failed to obtain ClassLoader from activity");
        env->DeleteLocalRef(activityClass);
        if (attachedHere) g_vm->DetachCurrentThread();
        return;
    }

    jclass classLoaderClass = env->FindClass("java/lang/ClassLoader");
    if (!classLoaderClass) {
        LOG_ERROR("Failed to find ClassLoader class");
        env->DeleteLocalRef(activityClass);
        env->DeleteLocalRef(classLoader);
        if (attachedHere) g_vm->DetachCurrentThread();
        return;
    }

    jmethodID loadClassMethod = env->GetMethodID(classLoaderClass, "loadClass", "(Ljava/lang/String;)Ljava/lang/Class;");
    if (!loadClassMethod) {
        LOG_ERROR("Failed to get loadClass method");
        env->DeleteLocalRef(activityClass);
        env->DeleteLocalRef(classLoader);
        env->DeleteLocalRef(classLoaderClass);
        if (attachedHere) g_vm->DetachCurrentThread();
        return;
    }

    // Nombre de la clase con puntos
    const char* className = "com.meta.haptics_sdk_example.JNIBridge";
    jstring jClassName = env->NewStringUTF(className);

    jobject localClassObj = env->CallObjectMethod(classLoader, loadClassMethod, jClassName);
    env->DeleteLocalRef(jClassName);
    env->DeleteLocalRef(classLoaderClass);
    env->DeleteLocalRef(classLoader);
    env->DeleteLocalRef(activityClass);

    if (env->ExceptionCheck() || !localClassObj) {
        env->ExceptionDescribe();
        env->ExceptionClear();
        LOG_ERROR("Failed to load JNIBridge class via ClassLoader");
        if (attachedHere) g_vm->DetachCurrentThread();
        return;
    }

    jclass localClass = static_cast<jclass>(localClassObj);

    // Crear referencia global a la clase
    g_jniBridgeClass = static_cast<jclass>(env->NewGlobalRef(localClass));
    env->DeleteLocalRef(localClass);

    if (!g_jniBridgeClass) {
        LOG_ERROR("Failed to create global ref for JNIBridge");
        if (attachedHere) g_vm->DetachCurrentThread();
        return;
    }

    // Obtener el método estático con la firma exacta
    g_onHapticEventMethod = env->GetStaticMethodID(
            g_jniBridgeClass,
            "onHapticEventFromNative",
            "(Ljava/lang/String;Ljava/lang/String;F)V"
    );

    if (env->ExceptionCheck() || !g_onHapticEventMethod) {
        env->ExceptionDescribe();
        env->ExceptionClear();
        LOG_ERROR("Failed to find onHapticEventFromNative method");
    } else {
        LOG_INFO("NetworkBridge initialized successfully");
    }

    if (attachedHere) {
        g_vm->DetachCurrentThread();
    }
}

void NetworkBridge::callJavaMethod(const std::string& eventType,
                                   const std::string& controller,
                                   float intensity) {
    if (!g_vm || !g_jniBridgeClass || !g_onHapticEventMethod) {
        LOG_ERROR("NetworkBridge not properly initialized");
        return;
    }

    JNIEnv* env = nullptr;
    bool attachedHere = false;
    if (!attachIfNeeded(g_vm, &env, &attachedHere)) {
        LOG_ERROR("Failed to get JNIEnv in callJavaMethod");
        return;
    }

    jstring jEventType = env->NewStringUTF(eventType.c_str());
    jstring jController = env->NewStringUTF(controller.c_str());

    // Pasar argumentos con jvalue para evitar problemas de varargs
    jvalue args[3];
    args[0].l = jEventType;
    args[1].l = jController;
    args[2].f = static_cast<jfloat>(intensity);

    env->CallStaticVoidMethodA(g_jniBridgeClass, g_onHapticEventMethod, args);

    if (env->ExceptionCheck()) {
        env->ExceptionDescribe();
        env->ExceptionClear();
        LOG_ERROR("Exception occurred when calling Java method");
    }

    env->DeleteLocalRef(jEventType);
    env->DeleteLocalRef(jController);

    if (attachedHere) {
        g_vm->DetachCurrentThread();
    }
}

void NetworkBridge::sendHapticEvent(const std::string& eventType,
                                    const std::string& controller,
                                    float intensity) {
    callJavaMethod(eventType, controller, intensity);
}

void NetworkBridge::setServerUrl(const std::string& url) {
    g_serverUrl = url;

    if (!g_vm) {
        LOG_ERROR("VM is null in setServerUrl");
        return;
    }

    JNIEnv* env = nullptr;
    bool attachedHere = false;
    if (!attachIfNeeded(g_vm, &env, &attachedHere)) {
        LOG_ERROR("Failed to get JNIEnv in setServerUrl");
        return;
    }

    if (g_jniBridgeClass) {
        jmethodID setUrlMethod = env->GetStaticMethodID(g_jniBridgeClass, "setServerUrl", "(Ljava/lang/String;)V");
        if (setUrlMethod) {
            jstring jUrl = env->NewStringUTF(url.c_str());
            env->CallStaticVoidMethod(g_jniBridgeClass, setUrlMethod, jUrl);
            if (env->ExceptionCheck()) {
                env->ExceptionDescribe();
                env->ExceptionClear();
                LOG_ERROR("Exception occurred when calling setServerUrl");
            }
            env->DeleteLocalRef(jUrl);
        } else {
            LOG_ERROR("setServerUrl method not found on JNIBridge");
        }
    } else {
        LOG_ERROR("g_jniBridgeClass is null in setServerUrl");
    }

    if (attachedHere) {
        g_vm->DetachCurrentThread();
    }
}

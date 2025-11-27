# Haptics SDK OpenXR Example App

This folder contains a minimal app that demonstrates how to use the Haptics SDK
directly with OpenXR.

The app supports running on Meta Quest headsets directly as well as running on a
Windows PC and streaming to the headset with Oculus Link. The variant running on
the headset directly is called the "Android" variant here, and the variant
running on Windows is called the "Windows" variant.

The app is structured as a basic OpenXR + OpenGL application that generates
haptics when controller trigger buttons are pulled, using the Haptics SDK.
All integration points in the source code are marked with `[haptics_sdk]` tags.

## Android Variant

### Dependencies

The example app targets SDK 26 and is tested with Android 12.

You need to have the Android NDK installed on your machine (at least version
21.4.7075529), with an `ANDROID_NDK_HOME` environment variable defined that
points to your local NDK directory.

### Building and Running

Open this folder in Android Studio and use the `Run -> Run 'app'` menu item to
build and run the project.

Alternatively, you can build and run from the command line with:
```
./gradlew app:installDebug
adb shell am start -n "com.meta.haptics_sdk_example/android.app.NativeActivity" \
    -a android.intent.action.MAIN -c android.intent.category.LAUNCHER
```

## Windows Variant

### Dependencies

The [Oculus App](https://www.meta.com/en-gb/help/quest/articles/getting-started/getting-started-with-rift-s/install-oculus-pc-app/)
needs to be installed. In `Settings -> General -> OpenXR Runtime`, Oculus needs
to be set as the active runtime.

For building, the MSVC compiler is required, which is available as part of the
MS Build Tools that can be downloaded from
[here](https://visualstudio.microsoft.com/downloads/). We support the MSVC
compiler shipped with Visual Studio 2022 and later.

### Building and Running

Open this folder in Visual Studio and use the `Debug -> Start Debugging` menu
item to build and run the project.

Alternatively, you can build and run from the command line with:
```
mkdir -p build
cd build
cmake ..
cmake --build . --config Debug
Debug/haptics_sdk_example.exe
```

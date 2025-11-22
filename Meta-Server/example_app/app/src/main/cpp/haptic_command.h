//
// Created by jaime on 20/11/2025.
//

#ifndef HAPTICS_SDK_OPENXR_EXAMPLE_HAPTIC_COMMAND_H
#define HAPTICS_SDK_OPENXR_EXAMPLE_HAPTIC_COMMAND_H

#include <string>

struct HapticCommand {
    float intensity;    // 1-10
    float duration;     // ms
    std::string pattern; // "continuous", "pulse", etc.

    HapticCommand() : intensity(5.0f), duration(1000.0f), pattern("continuous") {}
};

#endif //HAPTICS_SDK_OPENXR_EXAMPLE_HAPTIC_COMMAND_H

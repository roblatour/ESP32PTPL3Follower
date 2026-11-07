#pragma once
// Host-test stand-in for the ESP-IDF generated sdkconfig.h. Simulates the ESP32-P4
// target unless a runner predefines another supported target macro.
#if !defined(CONFIG_IDF_TARGET_ESP32P4) && !defined(CONFIG_IDF_TARGET_ESP32S31)
#define CONFIG_IDF_TARGET_ESP32P4 1
#endif

// Bluepad32 build-time configuration for the Pico 2 W hexapod firmware (part 02).
//
// Bluepad32's pico build emulates ESP-IDF "menuconfig" through this header: its
// sources #include "sdkconfig.h" off the include path (wired up in CMakeLists,
// alongside btstack_config.h). Values mirror Bluepad32's own
// examples/pico_w/src/sdkconfig.h — keep them in sync if the vendored Bluepad32
// is bumped (see BLUEPAD32_REF in sim.Dockerfile).
#ifndef HEXA_SDKCONFIG_H
#define HEXA_SDKCONFIG_H

#define CONFIG_BLUEPAD32_MAX_DEVICES 4
#define CONFIG_BLUEPAD32_MAX_ALLOWLIST 4
#define CONFIG_BLUEPAD32_GAP_SECURITY 1
#define CONFIG_BLUEPAD32_ENABLE_BLE_BY_DEFAULT 1
// #define CONFIG_BLUEPAD32_ENABLE_VIRTUAL_DEVICE_BY_DEFAULT 1

// Custom uni_platform (bt_teleop.cpp registers it via uni_platform_set_custom).
#define CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#define CONFIG_TARGET_PICO_W

// 2 == Info
#define CONFIG_BLUEPAD32_LOG_LEVEL 2

#endif  // HEXA_SDKCONFIG_H

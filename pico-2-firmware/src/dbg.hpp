// USB-CDC debug logging gate (enable_usb_debugging flag).
//
// All debug output goes through HEXA_DBG(...) and boot brings USB stdio up via
// HEXA_DBG_INIT(). The build option HEXA_ENABLE_USB_DEBUGGING (CMake, default ON)
// selects whether either does anything.
//
// Why gate it: printf writes to the USB-CDC endpoint, which BLOCKS when the host
// isn't draining the buffer — a multi-millisecond stall that jitters the 200 Hz
// control tick (and, via bt_teleop's callbacks, the cyw43 background BT
// servicing). With the flag OFF the writes and the USB stack are both gone, so
// the control loop runs jitter-free on the real robot. Leave it ON for bench
// development where the serial log is wanted.
//
// The OFF form keeps `if (false) printf(...)`: the call is dead-code-eliminated
// (no code, no USB), but the compiler still type-checks the format string and
// counts the arguments as used, so values logged ONLY in the heartbeat / joy
// dump (compute timings, heartbeat counter, heap low-water mark) don't trip
// -Wunused when logging is compiled out.
#pragma once

#include <cstdio>

#include "pico/stdlib.h"  // stdio_init_all

#if HEXA_ENABLE_USB_DEBUGGING
#  define HEXA_DBG(...)   std::printf(__VA_ARGS__)
#  define HEXA_DBG_INIT() stdio_init_all()
#else
#  define HEXA_DBG(...)   do { if (false) std::printf(__VA_ARGS__); } while (0)
#  define HEXA_DBG_INIT() ((void)0)
#endif

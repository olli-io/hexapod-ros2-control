#pragma once

#include <stdint.h>

#include "Expression.h"

// 1-bit eye rasterizer, a verbatim port of the browser prototype. No u8g2 or
// ESP-IDF dependencies: pixels are emitted through a callback so the same
// code runs on-device (u8g2_DrawPixel) and in host tests (bit array).
namespace eyes {

constexpr int kW  = 256;
constexpr int kH  = 64;
constexpr int kCY = 32;   // vertical eye center; lid squash pivots here

constexpr int kEyeLX = 78;
constexpr int kEyeRX = 178;

// Max gaze offset in px at full deflection (dx/dy = ±1).
constexpr int kGazeMaxX = 20;
constexpr int kGazeMaxY = 12;

// Receives one pixel, already clipped to [0,kW)×[0,kH).
using PixelSink = void (*)(void* ctx, int x, int y);

// Draw one eye centered at cx. lid: 1 = open, ~0.08 = shut (blink squash
// toward kCY). (gx,gy) is the gaze offset, applied at the pixel level so it
// composes with the lid transform. rightEye only matters for ANGRY (mirrored).
void drawEye(Expression e, bool rightEye, int cx, float lid, int gx, int gy,
             PixelSink sink, void* ctx);

}  // namespace eyes

#include "EyeRaster.h"

#include <math.h>

namespace {

constexpr float kPi = 3.14159265358979f;

struct Pt {
    float x, y;
};

// Stroke/fill primitives mirroring the JS prototype one-to-one. The gaze
// offset is added in setpx (pixel level); the lid squash is applied to
// control points via T() before curve generation.
struct Raster {
    eyes::PixelSink sink;
    void*           ctx;
    int             gx, gy;

    void setpx(int x, int y) const {
        x += gx;
        y += gy;
        if (x >= 0 && x < eyes::kW && y >= 0 && y < eyes::kH) sink(ctx, x, y);
    }

    // ~3px round-ish stroke brush.
    void dot(float fx, float fy) const {
        const int x = static_cast<int>(lroundf(fx));
        const int y = static_cast<int>(lroundf(fy));
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) setpx(x + dx, y + dy);
    }

    void seg(Pt a, Pt b) const {
        const float dx = b.x - a.x, dy = b.y - a.y;
        const int   n  = static_cast<int>(fmaxf(1.0f, ceilf(hypotf(dx, dy))));
        for (int i = 0; i <= n; ++i)
            dot(a.x + dx * i / n, a.y + dy * i / n);
    }

    void poly(const Pt* p, int n) const {
        for (int i = 0; i < n - 1; ++i) seg(p[i], p[i + 1]);
    }

    void ring(float cx, float cy, float rx, float ry) const {
        ry = fmaxf(0.5f, ry);
        const int n = static_cast<int>(ceilf((rx + ry) * 1.7f));
        for (int i = 0; i < n; ++i) {
            const float a = i * 2.0f * kPi / n;
            dot(cx + rx * cosf(a), cy + ry * sinf(a));
        }
    }

    void fillEllipse(float cx, float cy, float rx, float ry) const {
        ry = fmaxf(0.5f, ry);
        for (int y = -static_cast<int>(ceilf(ry)); y <= ry; ++y)
            for (int x = -static_cast<int>(ceilf(rx)); x <= rx; ++x)
                if ((x * x) / (rx * rx) + (y * y) / (ry * ry) <= 1.0f)
                    setpx(static_cast<int>(cx + x), static_cast<int>(cy + y));
    }

    void fillTri(Pt a, Pt b, Pt c) const {
        const int minx = static_cast<int>(floorf(fminf(a.x, fminf(b.x, c.x))));
        const int maxx = static_cast<int>(ceilf(fmaxf(a.x, fmaxf(b.x, c.x))));
        const int miny = static_cast<int>(floorf(fminf(a.y, fminf(b.y, c.y))));
        const int maxy = static_cast<int>(ceilf(fmaxf(a.y, fmaxf(b.y, c.y))));
        for (int y = miny; y <= maxy; ++y)
            for (int x = minx; x <= maxx; ++x) {
                const float w0 = (b.x - x) * (c.y - y) - (c.x - x) * (b.y - y);
                const float w1 = (c.x - x) * (a.y - y) - (a.x - x) * (c.y - y);
                const float w2 = (a.x - x) * (b.y - y) - (b.x - x) * (a.y - y);
                if ((w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                    (w0 <= 0 && w1 <= 0 && w2 <= 0))
                    setpx(x, y);
            }
    }
};

// Quadratic bezier, n=22 steps as in the prototype. Returns point count.
constexpr int kQuadN = 22;
int quad(Pt p0, Pt p1, Pt p2, Pt* out) {
    for (int i = 0; i <= kQuadN; ++i) {
        const float t = static_cast<float>(i) / kQuadN, u = 1.0f - t;
        out[i] = {u * u * p0.x + 2 * u * t * p1.x + t * t * p2.x,
                  u * u * p0.y + 2 * u * t * p1.y + t * t * p2.y};
    }
    return kQuadN + 1;
}

// Lid squash: scale a logical y toward the eye centerline.
Pt T(float x, float y, float lid) {
    return {x, eyes::kCY + (y - eyes::kCY) * lid};
}

}  // namespace

namespace eyes {

void drawEye(Expression e, bool rightEye, int cx, float lid, int gx, int gy,
             PixelSink sink, void* ctx) {
    const Raster r{sink, ctx, gx, gy};
    const float  fcx = static_cast<float>(cx);
    Pt           pts[kQuadN + 1];

    switch (e) {
        case Expression::HAPPY: {  // ^ ^
            const int n = quad(T(fcx - 22, 40, lid), T(fcx, 12, lid),
                               T(fcx + 22, 40, lid), pts);
            r.poly(pts, n);
            break;
        }
        case Expression::NEUTRAL:  // 0 0
            r.ring(fcx, kCY, 18, 18 * lid);
            break;
        case Expression::SLEEPY: {  // - -
            const Pt line[2] = {T(fcx - 22, 32, lid), T(fcx + 22, 32, lid)};
            r.poly(line, 2);
            break;
        }
        case Expression::DEAD: {  // x x
            const Pt d1[2] = {T(fcx - 15, 17, lid), T(fcx + 15, 47, lid)};
            const Pt d2[2] = {T(fcx + 15, 17, lid), T(fcx - 15, 47, lid)};
            r.poly(d1, 2);
            r.poly(d2, 2);
            break;
        }
        case Expression::GREEDY: {  // $ $
            constexpr float rad = 8;
            constexpr int   n   = 36;
            Pt top[n + 1], bot[n + 1];
            for (int i = 0; i <= n; ++i) {  // 270° arc, 0° → −270°
                const float a  = (-270.0f * i / n) * kPi / 180.0f;
                const float px = fcx + rad * cosf(a);
                const float py = (32 - rad) + rad * sinf(a);
                top[i] = T(px, py, lid);                  // upper bowl, opens right
                bot[i] = T(2 * fcx - px, 64 - py, lid);   // point-mirrored lower bowl
            }
            r.poly(top, n + 1);
            r.poly(bot, n + 1);
            const Pt bar[2] = {T(fcx, 32 - 2 * rad - 4, lid),
                               T(fcx, 32 + 2 * rad + 4, lid)};
            r.poly(bar, 2);
            break;
        }
        case Expression::WOOZY: {  // ~ ~
            int n = quad(T(fcx - 22, 32, lid), T(fcx - 11, 21, lid),
                         T(fcx, 32, lid), pts);
            r.poly(pts, n);
            n = quad(T(fcx, 32, lid), T(fcx + 11, 43, lid),
                     T(fcx + 22, 32, lid), pts);
            r.poly(pts, n);
            break;
        }
        case Expression::ANGRY: {  // > <
            const float s    = rightEye ? -1.0f : 1.0f;
            const Pt    v[3] = {T(fcx - s * 15, 17, lid), T(fcx + s * 15, 32, lid),
                                T(fcx - s * 15, 47, lid)};
            r.poly(v, 3);
            break;
        }
        case Expression::LOVE: {  // ♥ ♥
            const float ly = kCY + (25 - kCY) * lid;
            const float ry = 8 * lid;
            r.fillEllipse(fcx - 8, ly, 8, ry);
            r.fillEllipse(fcx + 8, ly, 8, ry);
            r.fillTri(T(fcx - 16, 28, lid), T(fcx + 16, 28, lid),
                      T(fcx, 49, lid));
            break;
        }
        default:
            break;
    }
}

}  // namespace eyes

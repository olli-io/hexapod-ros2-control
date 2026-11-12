#include "eyes.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

const std::vector<std::string> kMoods = {"normal", "happy", "angry", "sleepy", "dead",
                                         "woozy", "love", "scanning", "thug-life"};
const std::vector<std::string> kScanning = {"scanning",       "scanning-ccw",
                                            "scanning-counter", "snake-chase",
                                            "hop-chase",      "snake-infinity",
                                            "hop-infinity"};

namespace {

constexpr int S = kSS;

// Python-style modulo: the result has the sign of b.
double pymod(double a, double b) {
    double r = std::fmod(a, b);
    if (r != 0 && ((r < 0) != (b < 0))) r += b;
    return r;
}

// Point at fraction t [0,1) along a rounded rectangle, clockwise from top-centre.
Pt rrect_point(double cx, double cy, double w, double h, double r, double t) {
    double q = M_PI * r / 2;
    double sw = w - 2 * r, sh = h - 2 * r;
    struct Leg {
        char kind;
        double n;
    };
    const Leg legs[] = {{'l', sw / 2}, {'a', q}, {'l', sh}, {'a', q}, {'l', sw},
                        {'a', q},      {'l', sh}, {'a', q}, {'l', sw / 2}};
    double total = 0;
    for (const Leg& l : legs) total += l.n;
    double d = pymod(t, 1.0) * total;
    double x = cx, y = cy - h / 2, ang = 0.0;   // heading east
    for (const Leg& l : legs) {
        if (d <= l.n) {
            if (l.kind == 'l') return {x + std::cos(ang) * d, y + std::sin(ang) * d};
            double a = d / r;
            double ox = x + std::cos(ang + M_PI / 2) * r;
            double oy = y + std::sin(ang + M_PI / 2) * r;
            double th = ang - M_PI / 2 + a;
            return {ox + std::cos(th) * r, oy + std::sin(th) * r};
        }
        d -= l.n;
        if (l.kind == 'l') {
            x += std::cos(ang) * l.n;
            y += std::sin(ang) * l.n;
        } else {
            double ox = x + std::cos(ang + M_PI / 2) * r;
            double oy = y + std::sin(ang + M_PI / 2) * r;
            ang += M_PI / 2;
            double th = ang - M_PI / 2;
            x = ox + std::cos(th) * r;
            y = oy + std::sin(th) * r;
        }
    }
    return {x, y};
}

// ---------------------------------------------------------------- scanning

// Spin direction per eye side: +1 clockwise, -1 counter-clockwise.
// Returns false if `mood` is not one of the plain spinning sub moods.
bool scan_dirs(const std::string& mood, int side, int& dir) {
    if (mood == "scanning") dir = 1;
    else if (mood == "scanning-ccw") dir = -1;
    else if (mood == "scanning-counter") dir = side < 0 ? 1 : -1;
    else return false;
    return true;
}

// Hollow rounded-rect eye with a bright arc chasing round the rim.
void draw_scanning(Image& im, double x, double y, double w, double h, double r,
                   double phase, int direction) {
    double rim = 5 * S;
    gfx::outline_rounded_rect(im, x - w / 2, y - h / 2, x + w / 2, y + h / 2, r, rim, 70);
    const int n = 90;
    const double sweep = 0.30;
    std::vector<Pt> pts;
    for (int i = 0; i <= n; ++i)
        pts.push_back(rrect_point(x, y, w - rim, h - rim, r - rim / 2,
                                  direction * (phase + sweep * i / n)));
    gfx::polyline(im, pts, rim, 255, true);
}

// ------------------------------------------------------------------- chase

// One bright arc orbits an eye counter-clockwise for more than a full
// revolution, entering along the top plane and leaving along the bottom plane,
// then crosses the gap along the bottom plane into the other eye, orbits it
// the same way, and returns along its top plane. kChaseStageS is the time
// spent per eye.
// snake-chase: the arc visibly crosses the gap.
// hop-chase: the arc vanishes into the end of the orbit and reappears at the
// start of the other eye's orbit, nothing is drawn in the gap.
// The -infinity versions orbit the left eye counter-clockwise and the right
// eye clockwise, so the path crosses itself in the gap like a figure of eight.
constexpr double kChaseStageS = 3.0;

struct Eye {
    double x, y, w, h, r;   // supersampled px
};

bool is_chase(const std::string& mood) {
    return mood == "snake-chase" || mood == "hop-chase" || mood == "snake-infinity" ||
           mood == "hop-infinity";
}

double dist(Pt a, Pt b) { return std::hypot(a.x - b.x, a.y - b.y); }

// Sub-polyline of pts between arc-length a and b (pixels along it).
std::vector<Pt> window(const std::vector<Pt>& pts, double a, double b) {
    std::vector<Pt> out;
    double acc = 0.0;
    for (size_t k = 0; k + 1 < pts.size(); ++k) {
        Pt p = pts[k], q = pts[k + 1];
        double seg = dist(p, q);
        if (seg > 0 && acc + seg >= a && acc <= b) {
            double f0 = std::max(0.0, (a - acc) / seg);
            double f1 = std::min(1.0, (b - acc) / seg);
            Pt p0 = {p.x + (q.x - p.x) * f0, p.y + (q.y - p.y) * f0};
            Pt p1 = {p.x + (q.x - p.x) * f1, p.y + (q.y - p.y) * f1};
            if (out.empty()) out.push_back(p0);
            out.push_back(p1);
        }
        acc += seg;
    }
    return out;
}

// Fraction along the rim (as in rrect_point) of the point where a straight
// edge meets a corner: code is "tr", "br", "bl" or "tl".
double tangent_t(double iw, double ih, double rr, const char* code) {
    double q = M_PI * rr / 2, sw = iw - 2 * rr, sh = ih - 2 * rr;
    double total = 2 * sw + 4 * q + 2 * sh;
    bool top = code[0] == 't', right = code[1] == 'r';
    if (top && right) return sw / 2 / total;
    if (!top && right) return (sw / 2 + 2 * q + sh) / total;
    if (!top && !right) return (3 * sw / 2 + 2 * q + sh) / total;
    return (3 * sw / 2 + 4 * q + 2 * sh) / total;
}

// Rim points for `turns` laps: counter-clockwise if sign=1, clockwise if -1.
std::vector<Pt> orbit(double x, double y, double iw, double ih, double rr, double t_start,
                      double turns, int sign) {
    const int n = 180;
    int m = int(n * turns) + 1;
    std::vector<Pt> out;
    for (int i = 0; i <= m; ++i)
        out.push_back(rrect_point(x, y, iw, ih, rr, t_start - sign * turns * i / m));
    return out;
}

void draw_chase(Image& im, const std::vector<Eye>& eyes, double t, bool hop,
                bool infinity) {
    double rim = 5 * S;
    for (const Eye& e : eyes)
        gfx::outline_rounded_rect(im, e.x - e.w / 2, e.y - e.h / 2, e.x + e.w / 2,
                                  e.y + e.h / 2, e.r, rim, 70);
    const char* enters[2] = {"tr", infinity ? "tl" : "bl"};
    const char* leaves[2] = {"br", infinity ? "bl" : "tl"};
    const int signs[2] = {1, infinity ? -1 : 1};
    std::vector<Pt> loop;
    std::vector<std::vector<Pt>> orbs;
    for (size_t k = 0; k < eyes.size(); ++k) {
        const Eye& e = eyes[k];
        double iw = e.w - rim, ih = e.h - rim, rr = e.r - rim / 2;
        double t_in = tangent_t(iw, ih, rr, enters[k]);
        double t_out = tangent_t(iw, ih, rr, leaves[k]);
        double turns = 1 + pymod((t_in - t_out) * signs[k], 1.0);
        orbs.push_back(orbit(e.x, e.y, iw, ih, rr, t_in, turns, signs[k]));
        loop.insert(loop.end(), orbs.back().begin(), orbs.back().end());
    }
    loop.push_back(loop.front());   // straight runs across the gap close it
    double total = 0;
    for (size_t k = 0; k + 1 < loop.size(); ++k) total += dist(loop[k], loop[k + 1]);
    double sweep = 0.30 * total / 2.5;
    std::vector<Pt> arc;
    if (hop) {
        double u = pymod(t / kChaseStageS, 2.0);
        int idx = int(u);
        const std::vector<Pt>& pts = orbs[idx];
        double plen = 0;
        for (size_t k = 0; k + 1 < pts.size(); ++k) plen += dist(pts[k], pts[k + 1]);
        double head = (u - idx) * (plen + sweep);
        arc = window(pts, head - sweep, head);
    } else {
        double head = pymod(t / (2 * kChaseStageS), 1.0) * total;
        std::vector<Pt> twice = loop;
        twice.insert(twice.end(), loop.begin() + 1, loop.end());
        arc = window(twice, head + total - sweep, head + total);
    }
    if (arc.size() > 1) gfx::polyline(im, arc, rim, 255, true);
}

// -------------------------------------------------------------- cut-outs

void cut_dead(Image& im, double x, double y, double h) {
    double a = h * 0.27;
    for (int sgn : {1, -1})
        gfx::polyline(im, {{x - a, y - sgn * a}, {x + a, y + sgn * a}}, int(5 * S), 0,
                      false);
}

// Heartbeat: a strong beat at phase 0 and a weaker one at 0.18, each a fast
// jump to full size then an exponential decay, then rest until the next cycle.
constexpr double kBeats[2][2] = {{0.0, 1.0}, {0.18, 0.6}};
constexpr double kAttack = 0.05, kDecay = 0.12;
constexpr double kRest = 0.8, kPeak = 0.45;

double thump(double phase) {
    double env = 0.0;
    for (const auto& b : kBeats) {
        double dt = phase - b[0];
        if (dt < 0) continue;
        double v = b[1] * (dt < kAttack ? dt / kAttack : std::exp(-(dt - kAttack) / kDecay));
        env = std::max(env, v);
    }
    return kRest + kPeak * env;
}

void cut_love(Image& im, double x, double y, double h, double phase) {
    double k = h * 0.75 / 30 * thump(phase);
    std::vector<Pt> pts;
    for (int i = 0; i < 361; i += 6) {
        double t = i * M_PI / 180.0;
        double hx = 16 * std::pow(std::sin(t), 3);
        double hy = 13 * std::cos(t) - 5 * std::cos(2 * t) - 2 * std::cos(3 * t) -
                    std::cos(4 * t);
        pts.push_back({x + hx * k, y - (hy + 2.5) * k});
    }
    gfx::fill_polygon(im, pts, 0);
}

// ------------------------------------------------------------------ woozy

constexpr double kWoozyPeriod = 4.0, kWoozySwayPx = 6.0;

struct Woozy {
    double lid, dx, dy;
};

// One eye shuts as the other opens, drifting slowly back and forth, with a
// drunken sway. For the eye on `side` (-1 left, 1 right).
Woozy woozy_eye(int side, double t, double blink) {
    double s = 0.5 + 0.5 * std::sin(2 * M_PI * t / kWoozyPeriod);
    s = s * s * (3 - 2 * s);
    double openness = side < 0 ? s : 1 - s;
    double sway = kWoozySwayPx * std::sin(2 * M_PI * t / (kWoozyPeriod * 0.65));
    double droop = 3 * side * std::sin(2 * M_PI * t / kWoozyPeriod);
    return {(0.1 + 0.8 * openness) * blink, sway, droop};
}

// -------------------------------------------------------------- thug-life

// Pixel sunglasses drop in from the top, tilted, land with a bounce, sit for a
// while, then lift away again. Drawn once at rest, then rotated and shifted
// per frame.
constexpr int kGlassG = 6;                                     // one glasses "pixel"
constexpr int kLensRows[] = {16, 16, 16, 16, 16, 16, 14, 12};  // width per row, pixels
constexpr int kGlints[][2] = {{2, 1}, {4, 1}, {3, 2}, {5, 2}, {4, 3}, {6, 3}};
constexpr int kGlassTop = 8;                                   // screen y of lens top
constexpr double kThugCycle = 9.0;

struct Layers {
    Image col, mask;
    Rect box;   // where the mask is non-zero
};

const Layers& glasses_layers() {
    static const Layers layers = [] {
        const int g = kGlassG;
        struct Shape {
            bool rect;
            double a, b, c, d;   // rect: x0 y0 x1 y1; line: x0 y0 x1 y1
            double width;
        };
        std::vector<Shape> shapes;
        for (int cx : {66, 190}) {
            int row = 0;
            for (int n : kLensRows) {
                double y0 = kGlassTop + row * g;
                shapes.push_back({true, cx - n * g / 2.0, y0, cx + n * g / 2.0, y0 + g, 0});
                ++row;
            }
        }
        shapes.push_back({true, 114, double(kGlassTop), 142, double(kGlassTop + 2 * g), 0});
        shapes.push_back({false, 18, double(kGlassTop + 3), 0, double(kGlassTop - 3), 5});
        shapes.push_back({false, 238, double(kGlassTop + 3), 256, double(kGlassTop - 3), 5});
        Image col(kW * S, kH * S), mask(kW * S, kH * S);
        auto draw = [&](Image& im, double grow, uint8_t fill) {
            for (const Shape& s : shapes) {
                if (s.rect)
                    gfx::fill_rect(im, s.a * S - grow, s.b * S - grow, s.c * S + grow,
                                   s.d * S + grow, fill);
                else
                    gfx::polyline(im, {{s.a * S, s.b * S}, {s.c * S, s.d * S}},
                                  int(s.width * S + 2 * grow), fill, false);
            }
        };
        const double o = S;
        draw(mask, o, 255);
        draw(col, o, 100);   // grey outline so black shows on black
        draw(col, 0, 0);     // black body
        for (int cx : {66, 190})
            for (const auto& gl : kGlints) {
                double x0 = (cx - kLensRows[0] * g / 2.0 + gl[0] * g) * S;
                double y0 = (kGlassTop + gl[1] * g) * S;
                gfx::fill_rect(col, x0, y0, x0 + g * S - 1, y0 + g * S - 1, 255);
            }
        Rect box{kW * S, kH * S, 0, 0};
        for (int y = 0; y < mask.h; ++y)
            for (int x = 0; x < mask.w; ++x)
                if (mask.at(x, y)) {
                    box.x0 = std::min(box.x0, x);
                    box.y0 = std::min(box.y0, y);
                    box.x1 = std::max(box.x1, x + 1);
                    box.y1 = std::max(box.y1, y + 1);
                }
        return Layers{std::move(col), std::move(mask), box};
    }();
    return layers;
}

struct Pose {
    double dy, tilt;
};

// (dy px, tilt degrees) of the glasses at time t, or nothing while off screen.
std::optional<Pose> thug_pose(double t) {
    t = pymod(t, kThugCycle);
    if (t >= 1.2 && t < 1.7) {   // drop
        double p = (t - 1.2) / 0.5;
        return Pose{-(1 - p * p) * 70, 14 * (1 - p)};
    }
    if (t >= 1.7 && t < 7.5) {   // land, bounce, hold
        double q = (t - 1.7) / 0.6;
        if (q >= 1) return Pose{0.0, 0.0};
        return Pose{-6 * std::sin(M_PI * q) * (1 - q),
                    -4 * std::sin(2 * M_PI * q) * (1 - q)};
    }
    if (t >= 7.5 && t < 8.1) {   // lift away
        double p = (t - 7.5) / 0.6;
        return Pose{-(p * p) * 70, 8 * p};
    }
    return std::nullopt;
}

void paste_glasses(Image& im, double dx, double dy, double t) {
    auto pose = thug_pose(t);
    if (!pose) return;
    const Layers& l = glasses_layers();
    int ox = int(dx * S), oy = int((dy + pose->dy) * S);
    if (pose->tilt != 0) {
        double cx = kW * S / 2.0, cy = 32.0 * S;
        // where the corners of the layer's box land after the tilt
        double th = pose->tilt * M_PI / 180.0, c = std::cos(th), s = std::sin(th);
        double lo_x = 1e9, lo_y = 1e9, hi_x = -1e9, hi_y = -1e9;
        for (double px : {double(l.box.x0), double(l.box.x1)})
            for (double py : {double(l.box.y0), double(l.box.y1)}) {
                double rx = px - cx, ry = py - cy;
                double x = c * rx + s * ry + cx, y = -s * rx + c * ry + cy;
                lo_x = std::min(lo_x, x);
                hi_x = std::max(hi_x, x);
                lo_y = std::min(lo_y, y);
                hi_y = std::max(hi_y, y);
            }
        Rect rg{std::max(0, int(lo_x) - 2), std::max(0, int(lo_y) - 2),
                     std::min(kW * S, int(hi_x) + 3), std::min(kH * S, int(hi_y) + 3)};
        Image col(0, 0), mask(0, 0);
        gfx::rotate_bilinear_pair(l.col, l.mask, pose->tilt, cx, cy, rg, col, mask);
        gfx::paste_masked(im, col, mask, ox, oy, &rg);
    } else {
        gfx::paste_masked(im, l.col, l.mask, ox, oy, &l.box);
    }
}

}  // namespace

Image draw_eyes(double dx, double dy, double blink, const std::string& mood, double phase,
                double t) {
    Image im(kW * S, kH * S);
    const double ew = 78, eh_open = 46, rad = 16;
    std::vector<Eye> chase_eyes;
    for (int side : {-1, 1}) {
        double cx = side < 0 ? 66 : 190;
        double b = blink, ex = dx, ey = dy;
        if (mood == "woozy") {
            Woozy wz = woozy_eye(side, t, blink);
            b = wz.lid;
            ex = dx + wz.dx;
            ey = dy + wz.dy;
        }
        double eh = std::max(4.0, eh_open * b);
        double x = (cx + ex) * S, y = (32 + ey) * S;
        double w = ew * S, h = eh * S;
        double r = std::min(rad * S, h / 2);
        if (is_chase(mood)) {
            chase_eyes.push_back({x, y, w, h, r});
            continue;
        }
        int dir;
        if (scan_dirs(mood, side, dir)) {
            draw_scanning(im, x, y, w, h, r, phase, dir);
            continue;
        }
        gfx::fill_rounded_rect(im, x - w / 2, y - h / 2, x + w / 2, y + h / 2, r, 255);
        if (b < 0.5) continue;
        if (mood == "happy") {
            gfx::fill_ellipse(im, x - w * 0.62, y + h * 0.18, x + w * 0.62, y + h * 1.25, 0);
        } else if (mood == "angry") {
            int inner = -side;   // eye side facing the nose
            std::vector<Pt> pts = {{x - w / 2 - S, y - h / 2 - S},
                                   {x + w / 2 + S, y - h / 2 - S},
                                   {x + inner * w / 2 + inner * S, y - h / 2 + h * 0.5}};
            gfx::fill_polygon(im, pts, 0);
        } else if (mood == "dead") {
            cut_dead(im, x, y, h);
        } else if (mood == "love") {
            cut_love(im, x, y, h, phase);
        } else if (mood == "sleepy") {
            gfx::fill_rect(im, x - w / 2 - S, y - h / 2 - S, x + w / 2 + S,
                           y - h / 2 + h * 0.38, 0);
        }
    }
    if (mood == "thug-life") paste_glasses(im, dx, dy, t);
    if (!chase_eyes.empty())
        draw_chase(im, chase_eyes, t, mood.rfind("hop", 0) == 0,
                   mood.size() >= 8 && mood.compare(mood.size() - 8, 8, "infinity") == 0);
    return gfx::downscale_lanczos(im, S);
}

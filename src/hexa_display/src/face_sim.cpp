// Terminal eye emulator — a sim-only live mirror of the face.
//
// In the sim container there is no SH1122 OLED, so display_node runs headless:
// it computes every eye frame but the pixels go nowhere. This node is a second,
// on-demand sink of the same robot-state topics. It runs the *identical*
// expression/gaze/blink policy + EyeAnim pipeline as display_node.cpp (the pure
// logic is shared via hexa_display_support), but rasterizes the eyes into a
// terminal framebuffer painted with Unicode Braille blocks instead of
// driving SPI. Attach it to a running sim with `hexa sim face`; press `q` (or
// Ctrl-C) to detach — it is a sibling process of the sim launch, so closing it
// leaves the stack untouched.
//
// The policy half here is a deliberate copy of display_node.cpp's thin rclcpp
// glue (param decls + subscriptions + policyTick); the two must stay in sync.
// display_node.cpp is the robot-critical original and is not touched. Only the
// output half differs: terminal here, u8g2/SH1122 there.

#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>

#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <hexa_interfaces/msg/body_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "EyeAnim.h"
#include "EyeRaster.h"
#include "Expression.h"
#include "FaceNames.h"
#include "Sh1122Panel.h"

#include "expression_policy.hpp"
#include "face_animation.hpp"
#include "face_animation_runner.hpp"
#include "text_screen.hpp"

using hexa::display::BatteryMonitor;
using hexa::display::decide;
using hexa::display::DisplayTarget;
using hexa::display::FaceAnimation;
using hexa::display::FaceAnimationRunner;
using hexa::display::PolicyConfig;
using hexa::display::PolicyInputs;
using hexa::display::selectFaceAnimation;

namespace {

// EyeAnim's injected RNG — a process-wide mt19937, matching display_node.
uint32_t hostRand() {
    static std::mt19937 gen{std::random_device{}()};
    return gen();
}

std::string toLower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

uint32_t nowMs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// ---------------------------------------------------------------------------
// Terminal handling. Raw mode so single keystrokes reach us without Enter, with
// a robust restore path: atexit covers normal exit and rclcpp's SIGINT unwind;
// an explicit SIGTERM handler covers `docker stop` of the exec.
// ---------------------------------------------------------------------------
termios g_origTermios;
bool    g_termiosSaved = false;

void restoreTerminal() {
    if (!g_termiosSaved) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &g_origTermios);
    fputs("\x1b[?25h\x1b[?7h\x1b[0m\n", stdout);  // cursor, autowrap, color
    fflush(stdout);
}

void onSignal(int) {
    restoreTerminal();
    _exit(0);
}

void setupTerminal() {
    tcgetattr(STDIN_FILENO, &g_origTermios);
    g_termiosSaved = true;
    termios raw = g_origTermios;
    raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;  // non-blocking reads
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    atexit(restoreTerminal);
    signal(SIGTERM, onSignal);  // SIGINT is left to rclcpp (makes ok() false)
    fputs("\x1b[2J\x1b[?25l\x1b[?7l", stdout);  // clear, hide cursor, no autowrap
    fflush(stdout);
}

// ---------------------------------------------------------------------------
// Rendering. Even at max gaze (±20 px) plus the 3 px brush, the eyes stay inside
// x ∈ [34, 222], so only that 192 px window is shown (96 Braille columns).
// ---------------------------------------------------------------------------
constexpr int kCropX = 32;
constexpr int kCropW = 192;

uint8_t g_fb[eyes::kH][eyes::kW];

void sink(void*, int x, int y) { g_fb[y][x] = 1; }

int px(int x, int y) { return g_fb[y][x + kCropX]; }

// Each Braille glyph packs a 2×4 pixel block (8 dots) as U+2800 + bitmask, so
// the crop shrinks 2× horizontally and 4× vertically: kCropW/2 columns by
// kH/4 rows. Dot bit order is the Unicode Braille layout (columns then rows).
void drawFrame(const RenderState& state) {
    static char out[32 * 1024];
    char* p = out;
    p += sprintf(p, "\x1b[H");
    for (int cy = 0; cy < eyes::kH / 4; ++cy) {
        for (int cx = 0; cx < kCropW / 2; ++cx) {
            const int x = cx * 2, y = cy * 4;
            const int b = px(x, y) | px(x, y + 1) << 1 | px(x, y + 2) << 2 |
                          px(x + 1, y) << 3 | px(x + 1, y + 1) << 4 |
                          px(x + 1, y + 2) << 5 | px(x, y + 3) << 6 |
                          px(x + 1, y + 3) << 7;
            // UTF-8 for U+2800 + b
            *p++ = '\xE2';
            *p++ = static_cast<char>(0xA0 | (b >> 6));
            *p++ = static_cast<char>(0x80 | (b & 0x3F));
        }
        p += sprintf(p, "\x1b[K\n");
    }
    p += sprintf(p,
                 "\x1b[K EXPR %-7s GAZE %-10s "
                 "live mirror of display_node · space blink · q quit\x1b[J",
                 expressionName(state.expr), gazeName(state.gaze));
    fwrite(out, 1, static_cast<size_t>(p - out), stdout);
    fflush(stdout);
}

// Text-mode frame: the full 256x64 text panel as 128x16 Braille cells. Reads
// the headless panel buffer through textScreenPixel; the panel setup is
// U8G2_MIRROR (physical mount), so x is flipped back here.
void drawTextFrame(const u8g2_t* g) {
    auto tpx = [g](int x, int y) {
        return hexa::display::textScreenPixel(g, 255 - x, y) ? 1 : 0;
    };
    static char out[32 * 1024];
    char* p = out;
    p += sprintf(p, "\x1b[H");
    for (int cy = 0; cy < 64 / 4; ++cy) {
        for (int cx = 0; cx < 256 / 2; ++cx) {
            const int x = cx * 2, y = cy * 4;
            const int b = tpx(x, y) | tpx(x, y + 1) << 1 | tpx(x, y + 2) << 2 |
                          tpx(x + 1, y) << 3 | tpx(x + 1, y + 1) << 4 |
                          tpx(x + 1, y + 2) << 5 | tpx(x, y + 3) << 6 |
                          tpx(x + 1, y + 3) << 7;
            *p++ = '\xE2';
            *p++ = static_cast<char>(0xA0 | (b >> 6));
            *p++ = static_cast<char>(0x80 | (b & 0x3F));
        }
        p += sprintf(p, "\x1b[K\n");
    }
    p += sprintf(p, "\x1b[K TEXT MODE (/display/text, empty msg returns the "
                    "face) · q quit\x1b[J");
    fwrite(out, 1, static_cast<size_t>(p - out), stdout);
    fflush(stdout);
}

}  // namespace

// ---------------------------------------------------------------------------
// The policy half — a copy of display_node.cpp's rclcpp glue, minus the panel.
// Owns the EyeAnim + runner + RenderState target; the render loop pulls frames
// via frame(). Keep in sync with display_node.cpp.
// ---------------------------------------------------------------------------
class FaceSimNode : public rclcpp::Node {
public:
    FaceSimNode()
        : Node("face_sim"),
          _anim(&hostRand),
          _runner(
              [this](double lo, double hi) {
                  std::uniform_real_distribution<double> dist(lo, hi);
                  return dist(_rng);
              },
              [this](GazeDirection g) { _target.gaze = g; },
              [this]() { _anim.requestBlink(); }) {
        _rng.seed(std::random_device{}());

        const double update_rate_hz = declare_parameter<double>("update_rate_hz", 10.0);
        for (const auto& [state, expression] : hexa::display::defaultExpressionMap()) {
            declare_parameter<std::string>("expression_map." + state,
                                           toLower(expressionName(expression)));
        }
        declare_parameter<std::string>("animation_expression", "woozy");
        const std::string battery_topic = declare_parameter<std::string>(
            "battery_topic", "/hexa_hardware_aux/battery_state");
        declare_parameter<std::string>("battery_warning_expression", "sleepy");
        declare_parameter<std::string>("battery_critical_expression", "dead");
        declare_parameter<std::string>("scanning_expression", "scanning");
        declare_parameter<double>("battery_warning_v", 0.0);
        declare_parameter<double>("battery_critical_v", 0.0);
        declare_parameter<double>("battery_hysteresis_v", 0.3);
        declare_parameter<double>("battery_hold_s", 3.0);
        declare_parameter<double>("gaze_deadband", 0.15);
        declare_parameter<double>("gaze_exit_ratio", 0.6);
        declare_parameter<double>("gaze_wz_weight", 1.0);
        declare_parameter<double>("gaze_vy_max", 0.1);
        declare_parameter<double>("gaze_wz_max", 0.5);
        declare_parameter<double>("pose_pitch_threshold_rad", 0.08);
        declare_parameter<double>("pose_tilt_threshold_rad", 0.08);
        declare_parameter<double>("idling_start_delay_s", 4.0);

        for (const auto& [state, expression] : hexa::display::defaultExpressionMap()) {
            (void)expression;
            _config.expression_map[state] = parseExpressionParam("expression_map." + state);
        }
        _config.animation_expression = parseExpressionParam("animation_expression");
        _config.battery_warning_expression = parseExpressionParam("battery_warning_expression");
        _config.battery_critical_expression = parseExpressionParam("battery_critical_expression");
        _config.scanning_expression = parseExpressionParam("scanning_expression");
        _config.gaze_deadband = get_parameter("gaze_deadband").as_double();
        _config.gaze_exit_ratio = get_parameter("gaze_exit_ratio").as_double();
        _config.gaze_wz_weight = get_parameter("gaze_wz_weight").as_double();
        _config.gaze_vy_max = get_parameter("gaze_vy_max").as_double();
        _config.gaze_wz_max = get_parameter("gaze_wz_max").as_double();
        _config.pose_pitch_threshold_rad = get_parameter("pose_pitch_threshold_rad").as_double();
        _config.pose_tilt_threshold_rad = get_parameter("pose_tilt_threshold_rad").as_double();
        _config.idling_start_delay_s = get_parameter("idling_start_delay_s").as_double();

        _battery = BatteryMonitor(
            get_parameter("battery_warning_v").as_double(),
            get_parameter("battery_critical_v").as_double(),
            get_parameter("battery_hysteresis_v").as_double(),
            get_parameter("battery_hold_s").as_double());

        // transient_local to match hexa_locomotion's latched publisher: the boot
        // "folded" goes out before this node subscribes.
        rclcpp::QoS gait_qos(1);
        gait_qos.transient_local();
        _sub_gait = create_subscription<std_msgs::msg::String>(
            "/gait/state", gait_qos,
            [this](const std_msgs::msg::String& m) { _gait_state = m.data; });
        _sub_vel = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist& m) { _cmd_vel = m; });
        _sub_pose = create_subscription<hexa_interfaces::msg::BodyPose>(
            "/body/pose", 10,
            [this](const hexa_interfaces::msg::BodyPose& m) { _body_pose = m; });
        rclcpp::QoS animation_qos(1);
        animation_qos.transient_local();
        _sub_animation = create_subscription<std_msgs::msg::String>(
            "/animation/mode", animation_qos,
            [this](const std_msgs::msg::String& m) { _animation_mode = m.data; });
        _sub_battery = create_subscription<sensor_msgs::msg::BatteryState>(
            battery_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::BatteryState& m) { _battery_voltage = m.voltage; });
        rclcpp::QoS text_qos(1);
        text_qos.transient_local();
        _sub_text = create_subscription<std_msgs::msg::String>(
            "/display/text", text_qos,
            [this](const std_msgs::msg::String& m) { _display_text = m.data; });
        // Pairing state from hexa_buttons; transient_local, same as the panel node.
        rclcpp::QoS scanning_qos(1);
        scanning_qos.transient_local();
        _sub_scanning = create_subscription<std_msgs::msg::Bool>(
            "/bluetooth/scanning", scanning_qos,
            [this](const std_msgs::msg::Bool& m) { _bluetooth_scanning = m.data; });

        _policy_timer = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / update_rate_hz)),
            [this] { policyTick(); });
    }

    // Advance the renderer to nowMs and return the frame to draw.
    eyes::AnimFrame frame(uint32_t now_ms) { return _anim.update(_target, now_ms); }

    // For manual testing (space key).
    void requestBlink() { _anim.requestBlink(); }

    const RenderState& target() const { return _target; }

    // Latest /display/text message ("" = face mode).
    const std::string& displayText() const { return _display_text; }

private:
    Expression parseExpressionParam(const std::string& param) {
        const std::string value = get_parameter(param).as_string();
        if (auto e = face::parseExpression(value)) {
            return *e;
        }
        std::string valid;
        for (int i = 0; i < static_cast<int>(Expression::_COUNT); ++i) {
            if (i) valid += ", ";
            valid += toLower(expressionName(static_cast<Expression>(i)));
        }
        throw std::runtime_error(param + ": unknown expression '" + value +
                                 "'; valid: " + valid);
    }

    void policyTick() {
        const double now = get_clock()->now().seconds();
        BatteryMonitor::Flags battery;
        if (_battery_voltage) {
            battery = _battery.update(*_battery_voltage, now);
        }
        PolicyInputs inputs;
        inputs.gait_state = _gait_state;
        inputs.vx = _cmd_vel.linear.x;
        inputs.vy = _cmd_vel.linear.y;
        inputs.wz = _cmd_vel.angular.z;
        inputs.animation_mode = _animation_mode;
        inputs.roll = _body_pose.roll;
        inputs.pitch = _body_pose.pitch;
        inputs.yaw = _body_pose.yaw;
        inputs.battery_low = battery.low;
        inputs.battery_critical = battery.critical;
        inputs.bluetooth_scanning = _bluetooth_scanning;

        _last_target = decide(inputs, _config, _last_target);
        const FaceAnimation* animation = _runner.update(
            selectFaceAnimation(inputs, _config), now, _config.idling_start_delay_s);

        _target.expr = _last_target.expression;
        if (animation == nullptr) {
            _target.gaze = _last_target.gaze;
            _target.gazeEaseMs = 0;
            _target.gazeScale = 1.0f;
        } else {
            _runner.run(*animation, now);
            _target.gazeEaseMs =
                static_cast<uint32_t>(animation->gaze_ease_s() * 1000.0);
            _target.gazeScale = static_cast<float>(animation->gaze_scale());
        }
    }

    eyes::EyeAnim _anim;
    std::mt19937 _rng;
    RenderState _target{Expression::NEUTRAL, GazeDirection::CENTER};

    PolicyConfig _config;
    BatteryMonitor _battery;
    FaceAnimationRunner _runner;
    DisplayTarget _last_target = hexa::display::kIdleTarget;

    std::optional<std::string> _gait_state;
    geometry_msgs::msg::Twist _cmd_vel;
    hexa_interfaces::msg::BodyPose _body_pose;
    std::string _animation_mode;
    std::optional<double> _battery_voltage;
    std::string _display_text;
    bool _bluetooth_scanning = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_gait;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _sub_vel;
    rclcpp::Subscription<hexa_interfaces::msg::BodyPose>::SharedPtr _sub_pose;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_animation;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr _sub_battery;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_text;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_scanning;
    rclcpp::TimerBase::SharedPtr _policy_timer;
};

namespace {

// Drain pending keystrokes. Returns false to quit; `space` forces a blink.
bool pollInput(FaceSimNode& node) {
    uint8_t buf[16];
    const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
    for (ssize_t i = 0; i < n; ++i) {
        switch (buf[i]) {
            case 'q':
            case 'Q': return false;
            case ' ': node.requestBlink(); break;
            default: break;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceSimNode>();
    setupTerminal();

    int rc = 0;
    try {
        // Render loop: wall-clock ~30 fps, decoupled from the policy timer.
        // spin_some() services the subscriptions and the policy tick; the frame
        // is pulled from EyeAnim and painted to the terminal each iteration.
        // Text-mode render target: a headless panel, same u8g2 pipeline as
        // display_node's real one.
        face::Sh1122Panel textPanel;
        face::PanelConfig textCfg;
        textCfg.headless = true;
        textPanel.begin(textCfg);
        const hexa::display::TextScreenConfig textLayout;
        std::string drawnText;

        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            const std::string& text = node->displayText();
            if (!text.empty()) {
                if (text != drawnText) {
                    drawnText = text;
                    textPanel.clearBuffer();
                    hexa::display::drawTextScreen(textPanel.u8g2(), text, textLayout);
                }
                drawTextFrame(textPanel.u8g2());
            } else {
                drawnText.clear();
                const eyes::AnimFrame f = node->frame(nowMs());
                memset(g_fb, 0, sizeof(g_fb));
                eyes::drawEye(f.expr, false, eyes::kEyeLX, f.lid, f.gx, f.gy,
                              f.phase, sink, nullptr);
                eyes::drawEye(f.expr, true, eyes::kEyeRX, f.lid, f.gx, f.gy,
                              f.phase, sink, nullptr);
                drawFrame(node->target());
            }
            if (!pollInput(*node)) break;
            usleep(33000);
        }
    } catch (const std::exception& e) {
        restoreTerminal();
        fprintf(stderr, "face_sim fatal: %s\n", e.what());
        rc = 1;
    }
    restoreTerminal();
    rclcpp::shutdown();
    return rc;
}

// Hexapod "face" ROS2 node. One process maps robot state to an expression/gaze
// policy and rasterizes the eyes on a 256x64 SH1122 OLED directly from the Pi,
// reusing the firmware's platform-free eye animation core (EyeAnim + EyeRaster).
//
// A pure sink of robot state: it subscribes to the gait engine state, the body
// velocity command, the user body pose, the posture animation mode, the
// battery state, and the Bluetooth pairing state, and drives the renderer
// target directly — the policy half and the render half share one process, so
// there is no /display/* topic hop.
//
// Text mode: a non-empty /display/text message replaces the eyes with the
// message (Pixel Operator font, word-wrapped, centered); an empty message
// returns to the face. The policy keeps ticking underneath so the face resumes
// with current state.

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <hexa_interfaces/msg/body_pose.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "EyeAnim.h"
#include "EyeRaster.h"
#include "Expression.h"
#include "FaceNames.h"
#include "Sh1122Panel.h"
#include "u8g2.h"

#include "expression_policy.hpp"
#include "face_animation.hpp"
#include "face_animation_runner.hpp"
#include "text_screen.hpp"

namespace {

// EyeAnim's injected RNG. A process-wide mt19937, seeded once, satisfies the
// uint32_t(*)() signature (a plain function pointer cannot capture state).
uint32_t hostRand() {
    static std::mt19937 gen{std::random_device{}()};
    return gen();
}

void pixelSink(void* ctx, int x, int y) {
    u8g2_DrawPixel(static_cast<u8g2_t*>(ctx),
                   static_cast<u8g2_uint_t>(x), static_cast<u8g2_uint_t>(y));
}

// Exact equality is intended: drawEye is a pure function of these fields, so
// bit-identical frames rasterize to identical pixels. EyeAnim emits a constant
// frame while idle (lid == 1.0f, settled integer gaze, phase 0), so this
// reliably fires. SCANNING steps its phase in discrete jumps, so the spinner
// redraws once per step rather than once per render tick.
bool sameFrame(const eyes::AnimFrame& a, const eyes::AnimFrame& b) {
    return a.expr == b.expr && a.lid == b.lid && a.gx == b.gx && a.gy == b.gy &&
           a.phase == b.phase;
}

rcl_interfaces::msg::ParameterDescriptor readOnly(const std::string& desc) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = desc;
    d.read_only = true;
    return d;
}

std::string toLower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

using hexa::display::BatteryMonitor;
using hexa::display::decide;
using hexa::display::DisplayTarget;
using hexa::display::FaceAnimation;
using hexa::display::FaceAnimationRunner;
using hexa::display::PolicyConfig;
using hexa::display::PolicyInputs;
using hexa::display::selectFaceAnimation;

class DisplayNode : public rclcpp::Node {
public:
    DisplayNode()
        : Node("display_node"),
          _anim(&hostRand),
          _runner(
              [this](double lo, double hi) {
                  std::uniform_real_distribution<double> dist(lo, hi);
                  return dist(_rng);
              },
              [this](GazeDirection g) { _target.gaze = g; },
              [this]() { _anim.requestBlink(); }) {
        _rng.seed(std::random_device{}());

        // --- Panel / render params (read-only, as the renderer half) ---
        face::PanelConfig cfg;
        cfg.spi_device = declare_parameter<std::string>(
            "spi_device", cfg.spi_device, readOnly("spidev path for the SH1122"));
        cfg.spi_speed_hz = static_cast<uint32_t>(declare_parameter<int>(
            "spi_speed_hz", static_cast<int>(cfg.spi_speed_hz),
            readOnly("SPI clock (Hz)")));
        cfg.gpio_chip = declare_parameter<std::string>(
            "gpio_chip", cfg.gpio_chip, readOnly("GPIO character device"));
        cfg.dc_line = declare_parameter<int>("dc_line", 24, readOnly("DC GPIO line"));
        cfg.rst_line = declare_parameter<int>("rst_line", 25, readOnly("RST GPIO line"));
        cfg.cs_line = declare_parameter<int>(
            "cs_line", -1, readOnly("CS GPIO line, -1 to leave CS to the controller"));
        cfg.headless = declare_parameter<bool>(
            "headless", false, readOnly("run without touching SPI/GPIO"));
        const int render_hz = std::max(1, static_cast<int>(declare_parameter<int>(
            "render_hz", 60, readOnly("render loop rate (Hz)"))));

        // --- Policy params ---
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

        // Fail fast on expression-name typos in the YAML.
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

        if (!_panel.begin(cfg)) {
            RCLCPP_FATAL(get_logger(),
                         "SH1122 init failed (spi=%s chip=%s). Check wiring, "
                         "dtparam=spi=on, and access to spidev/gpiochip "
                         "(groups spi/gpio), or set headless:=true.",
                         cfg.spi_device.c_str(), cfg.gpio_chip.c_str());
            throw std::runtime_error("panel init failed");
        }

        // --- Inputs: cache the latest of each; the policy tick snapshots them ---
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
        // transient_local to match the teleop publisher, so a late display start
        // still sees the active animation mode.
        rclcpp::QoS animation_qos(1);
        animation_qos.transient_local();
        _sub_animation = create_subscription<std_msgs::msg::String>(
            "/animation/mode", animation_qos,
            [this](const std_msgs::msg::String& m) { _animation_mode = m.data; });
        _sub_battery = create_subscription<sensor_msgs::msg::BatteryState>(
            battery_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::BatteryState& m) { _battery_voltage = m.voltage; });
        // Text mode input. transient_local so a late display start still shows
        // the message; an empty string hands the panel back to the face.
        rclcpp::QoS text_qos(1);
        text_qos.transient_local();
        _sub_text = create_subscription<std_msgs::msg::String>(
            "/display/text", text_qos,
            [this](const std_msgs::msg::String& m) { _display_text = m.data; });
        // Pairing state from hexa_buttons. transient_local so a display restart
        // mid-scan comes back up wearing the spinners rather than the face.
        rclcpp::QoS scanning_qos(1);
        scanning_qos.transient_local();
        _sub_scanning = create_subscription<std_msgs::msg::Bool>(
            "/bluetooth/scanning", scanning_qos,
            [this](const std_msgs::msg::Bool& m) { _bluetooth_scanning = m.data; });
        // The other thing a front-panel hold starts: a network-mode switch.
        // Separate topic because it means something else, same spinners because
        // the face's answer to "wait, I am working" is the same either way.
        _sub_busy = create_subscription<std_msgs::msg::Bool>(
            "/display/busy", scanning_qos,
            [this](const std_msgs::msg::Bool& m) { _display_busy = m.data; });

        // Policy tick on the node clock (sim-time aware). The renderer eases gaze
        // and blinks autonomously, so a low rate suffices.
        _policy_timer = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / update_rate_hz)),
            [this] { policyTick(); });
        // Render loop: wall-clock, decoupled from the policy rate. Frames only
        // reach SPI when the pixels change.
        _render_timer = create_wall_timer(
            std::chrono::milliseconds(1000 / render_hz), [this] { renderTick(); });

        RCLCPP_INFO(get_logger(), "face up: 256x64 SH1122 on %s @ %d Hz%s",
                    cfg.spi_device.c_str(), render_hz,
                    cfg.headless ? " (headless)" : "");
    }

    ~DisplayNode() override { _panel.sleep(); }

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

    uint32_t nowMs() {
        using namespace std::chrono;
        if (_t0.time_since_epoch().count() == 0) _t0 = steady_clock::now();
        return static_cast<uint32_t>(
            duration_cast<milliseconds>(steady_clock::now() - _t0).count());
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
        inputs.busy = _bluetooth_scanning || _display_busy;

        _last_target = decide(inputs, _config, _last_target);
        const FaceAnimation* animation = _runner.update(
            selectFaceAnimation(inputs, _config), now, _config.idling_start_delay_s);

        // Expression always comes from the policy. Gaze too, unless a face
        // animation is active — then its own steps own the gaze (and its
        // gaze_ease_s the drift speed).
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

    void renderTick() {
        if (!_display_text.empty()) {
            // Text mode: the message is static, so redraw only on change (the
            // string compare replaces the eye path's sameFrame skip).
            if (_display_text != _drawn_text) {
                _drawn_text = _display_text;
                _panel.clearBuffer();
                hexa::display::drawTextScreen(_panel.u8g2(), _display_text, _text_cfg);
                _panel.present();
            }
            _haveFrame = false;  // repaint the face when text mode ends
            return;
        }
        _drawn_text.clear();

        const eyes::AnimFrame f = _anim.update(_target, nowMs());
        // Skip the float-heavy raster entirely when the animation frame is
        // unchanged — between blinks and once gaze has settled the face is
        // static, so there is nothing to redraw.
        if (_haveFrame && sameFrame(f, _lastFrame)) return;
        _lastFrame = f;
        _haveFrame = true;

        _panel.clearBuffer();
        u8g2_t* g = _panel.u8g2();
        eyes::drawEye(f.expr, false, eyes::kEyeLX, f.lid, f.gx, f.gy, f.phase,
                      pixelSink, g);
        eyes::drawEye(f.expr, true, eyes::kEyeRX, f.lid, f.gx, f.gy, f.phase,
                      pixelSink, g);
        _panel.present();  // flushes over SPI only if the pixels changed
    }

    // Render half.
    face::Sh1122Panel _panel;
    eyes::EyeAnim _anim;
    std::mt19937 _rng;
    RenderState _target{Expression::NEUTRAL, GazeDirection::CENTER};
    eyes::AnimFrame _lastFrame{};
    bool _haveFrame = false;
    std::chrono::steady_clock::time_point _t0{};
    hexa::display::TextScreenConfig _text_cfg;
    std::string _drawn_text;  // text currently on the panel ("" = face)

    // Policy half.
    PolicyConfig _config;
    BatteryMonitor _battery;
    FaceAnimationRunner _runner;
    DisplayTarget _last_target = hexa::display::kIdleTarget;

    // Cached inputs.
    std::optional<std::string> _gait_state;
    geometry_msgs::msg::Twist _cmd_vel;
    hexa_interfaces::msg::BodyPose _body_pose;
    std::string _animation_mode;
    std::optional<double> _battery_voltage;
    std::string _display_text;
    bool _bluetooth_scanning = false;
    bool _display_busy = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_gait;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _sub_vel;
    rclcpp::Subscription<hexa_interfaces::msg::BodyPose>::SharedPtr _sub_pose;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_animation;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr _sub_battery;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_text;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_scanning;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_busy;
    rclcpp::TimerBase::SharedPtr _policy_timer;
    rclcpp::TimerBase::SharedPtr _render_timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        rclcpp::spin(std::make_shared<DisplayNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("display_node"), "fatal: %s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}

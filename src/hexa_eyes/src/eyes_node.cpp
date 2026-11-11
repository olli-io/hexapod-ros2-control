// Hexapod "face" node, greyscale variant. One process maps robot state to the
// same expression/gaze policy hexa_display uses and draws the eyes with the
// 16-grey renderer from eye-animation-cpp (shared/eyes_core) on the 256x64
// SH1122 OLED. It is an alternative to hexa_display, never run beside it:
// hexa_bringup's face.yaml `backend` starts exactly one of the two.
//
// The topic inputs and the policy half are the same as display_node.cpp's, and
// share its parameters (display.yaml); only the render half differs. The shared
// eyes::EyeAnim still owns expression-change blinks, idle blinks and gaze
// easing, so the greyscale face moves like the 1-bit one. Text mode
// (/display/text) is drawn with the shared text screen into a headless u8g2
// buffer and sent through the same greyscale driver.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hexa_interfaces/msg/body_pose.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "EyeAnim.h"
#include "Expression.h"
#include "FaceNames.h"
#include "Sh1122Panel.h"
#include "sh1122.hpp"

#include "expression_policy.hpp"
#include "gesture_expressions.hpp"
#include "face_animation.hpp"
#include "face_animation_runner.hpp"
#include "text_screen.hpp"

#include "eyes_face.hpp"

namespace {

// EyeAnim's injected RNG. A process-wide mt19937, seeded once, satisfies the
// uint32_t(*)() signature (a plain function pointer cannot capture state).
uint32_t hostRand() {
    static std::mt19937 gen{std::random_device{}()};
    return gen();
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

// Mood each expression shows unless eyes.yaml says otherwise. Greedy has no
// counterpart in the 1-bit face's art, so it gets the extra mood.
const char* defaultMood(Expression e) {
    switch (e) {
        case Expression::HAPPY: return "happy";
        case Expression::SLEEPY: return "sleepy";
        case Expression::DEAD: return "dead";
        case Expression::GREEDY: return "thug-life";
        case Expression::WOOZY: return "woozy";
        case Expression::ANGRY: return "angry";
        case Expression::LOVE: return "love";
        case Expression::SCANNING: return "scanning";
        default: return "normal";
    }
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

class EyesNode : public rclcpp::Node {
public:
    EyesNode()
        : Node("eyes_node"),
          _anim(&hostRand),
          _runner(
              [this](double lo, double hi) {
                  std::uniform_real_distribution<double> dist(lo, hi);
                  return dist(_rng);
              },
              [this](GazeDirection g) { _target.gaze = g; },
              [this]() { _anim.requestBlink(); }) {
        _rng.seed(std::random_device{}());

        // --- Panel / render params. The pins come from display.yaml (same
        // panel, same wiring); eyes.yaml adds what only this renderer has. ---
        const std::string spi_device = declare_parameter<std::string>(
            "spi_device", "/dev/spidev0.0", readOnly("spidev path for the SH1122"));
        Sh1122Config cfg;
        if (std::sscanf(spi_device.c_str(), "/dev/spidev%d.%d", &cfg.port, &cfg.device) != 2) {
            throw std::runtime_error("spi_device: expected /dev/spidev<bus>.<cs>, got '" +
                                     spi_device + "'");
        }
        cfg.speed_hz = static_cast<uint32_t>(declare_parameter<int>(
            "spi_speed_hz", static_cast<int>(cfg.speed_hz), readOnly("SPI clock (Hz)")));
        cfg.gpiochip = declare_parameter<std::string>(
            "gpio_chip", cfg.gpiochip, readOnly("GPIO character device"));
        cfg.dc = declare_parameter<int>("dc_line", cfg.dc, readOnly("DC GPIO line"));
        cfg.rst = declare_parameter<int>("rst_line", cfg.rst, readOnly("RST GPIO line"));
        if (declare_parameter<int>("cs_line", -1, readOnly("CS GPIO line, -1 = controller")) >= 0) {
            throw std::runtime_error(
                "cs_line: the greyscale driver leaves chip select to the SPI controller; "
                "set cs_line to -1");
        }
        cfg.contrast = declare_parameter<int>("contrast", cfg.contrast, readOnly("panel contrast 0-255"));
        cfg.hflip = declare_parameter<bool>("hflip", cfg.hflip, readOnly("mirror horizontally"));
        cfg.vflip = declare_parameter<bool>("vflip", cfg.vflip, readOnly("flip vertically"));
        const bool headless = declare_parameter<bool>(
            "headless", false, readOnly("run without touching SPI/GPIO"));
        const int render_hz = std::max(1, static_cast<int>(declare_parameter<int>(
            "render_hz", 30, readOnly("render loop rate (Hz)"))));

        // --- Expression -> mood map ---
        std::array<std::string, hexa::eyes_cpp::kExpressionCount> moods;
        for (std::size_t i = 0; i < moods.size(); ++i) {
            const auto e = static_cast<Expression>(i);
            const std::string param = "expression_mood." + toLower(expressionName(e));
            moods[i] = declare_parameter<std::string>(param, defaultMood(e));
        }
        _face = std::make_unique<hexa::eyes_cpp::EyesFace>(moods);   // validates the names

        // --- Policy params: identical to display_node (same display.yaml) ---
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
        declare_parameter<std::string>("posture_tilt_expression", "happy");
        declare_parameter<std::string>("posture_shift_expression", "love");
        declare_parameter<std::string>("posture_both_expression", "angry");
        declare_parameter<double>("posture_tilt_threshold_rad", 0.03);
        declare_parameter<double>("posture_shift_threshold_m", 0.005);
        declare_parameter<double>("posture_exit_ratio", 0.6);
        declare_parameter<double>("battery_warning_v", 0.0);
        declare_parameter<double>("battery_critical_v", 0.0);
        declare_parameter<double>("battery_hysteresis_v", 0.3);
        declare_parameter<double>("battery_hold_s", 3.0);
        declare_parameter<double>("gaze_wz_weight", 1.0);
        declare_parameter<double>("gaze_vy_max", 0.1);
        declare_parameter<double>("gaze_wz_max", 0.5);
        declare_parameter<double>("pose_pitch_threshold_rad", 0.08);
        declare_parameter<double>("pose_tilt_threshold_rad", 0.08);
        declare_parameter<double>("pose_pitch_full_rad", 0.26);
        declare_parameter<double>("pose_tilt_full_rad", 0.35);
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
        _config.posture_tilt_expression = parseExpressionParam("posture_tilt_expression");
        _config.posture_shift_expression = parseExpressionParam("posture_shift_expression");
        _config.posture_both_expression = parseExpressionParam("posture_both_expression");
        _config.posture_tilt_threshold_rad =
            get_parameter("posture_tilt_threshold_rad").as_double();
        _config.posture_shift_threshold_m =
            get_parameter("posture_shift_threshold_m").as_double();
        _config.posture_exit_ratio = get_parameter("posture_exit_ratio").as_double();
        _config.gaze_wz_weight = get_parameter("gaze_wz_weight").as_double();
        _config.gaze_vy_max = get_parameter("gaze_vy_max").as_double();
        _config.gaze_wz_max = get_parameter("gaze_wz_max").as_double();
        _config.pose_pitch_threshold_rad = get_parameter("pose_pitch_threshold_rad").as_double();
        _config.pose_tilt_threshold_rad = get_parameter("pose_tilt_threshold_rad").as_double();
        _config.pose_pitch_full_rad = get_parameter("pose_pitch_full_rad").as_double();
        _config.pose_tilt_full_rad = get_parameter("pose_tilt_full_rad").as_double();
        _config.idling_start_delay_s = get_parameter("idling_start_delay_s").as_double();
        _config.gesture_expression_map = hexa::display::loadGestureExpressions(
            ament_index_cpp::get_package_share_directory("hexa_description") +
            "/config/gestures.yaml");

        _battery = BatteryMonitor(
            get_parameter("battery_warning_v").as_double(),
            get_parameter("battery_critical_v").as_double(),
            get_parameter("battery_hysteresis_v").as_double(),
            get_parameter("battery_hold_s").as_double());

        // Seed both targets with the policy's own boot answer, as display_node does,
        // so the face never flashes NEUTRAL on the way up.
        _last_target = decide(PolicyInputs{}, _config, _last_target);
        _target.expr = _last_target.expression;
        _target.gaze = _last_target.gaze;

        // The text screen draws into a u8g2 buffer that never touches the bus;
        // its pixels are copied into the greyscale frame.
        face::PanelConfig text_cfg;
        text_cfg.headless = true;
        if (!_text_panel.begin(text_cfg)) {
            throw std::runtime_error("text buffer init failed");
        }
        if (!headless) {
            try {
                _dev = std::make_unique<Sh1122>(cfg);
            } catch (const std::exception& e) {
                RCLCPP_FATAL(get_logger(),
                             "SH1122 init failed (spi=%s chip=%s): %s. Check wiring, "
                             "dtparam=spi=on, access to spidev/gpiochip (groups spi/gpio) "
                             "and that hexa_display is not running (face.yaml backend), or "
                             "set headless:=true.",
                             spi_device.c_str(), cfg.gpiochip.c_str(), e.what());
                throw;
            }
        }

        // --- Inputs: cache the latest of each; the policy tick snapshots them ---
        rclcpp::QoS gait_qos(1);
        gait_qos.transient_local();
        _sub_gait = create_subscription<std_msgs::msg::String>(
            "/gait/state", gait_qos,
            [this](const std_msgs::msg::String& m) { _gait_state = m.data; });
        _sub_gesture = create_subscription<std_msgs::msg::String>(
            "/gait/gesture", gait_qos,
            [this](const std_msgs::msg::String& m) { _gesture = m.data; });
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
        rclcpp::QoS scanning_qos(1);
        scanning_qos.transient_local();
        _sub_scanning = create_subscription<std_msgs::msg::Bool>(
            "/bluetooth/scanning", scanning_qos,
            [this](const std_msgs::msg::Bool& m) { _bluetooth_scanning = m.data; });
        _sub_busy = create_subscription<std_msgs::msg::Bool>(
            "/display/busy", scanning_qos,
            [this](const std_msgs::msg::Bool& m) { _display_busy = m.data; });

        _policy_timer = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / update_rate_hz)),
            [this] { policyTick(); });
        // Render loop: wall-clock. Most moods animate continuously, so every tick
        // draws; the driver sends only the rows that changed.
        _render_timer = create_wall_timer(
            std::chrono::milliseconds(1000 / render_hz), [this] { renderTick(); });

        RCLCPP_INFO(get_logger(), "greyscale face up: 256x64 SH1122 on %s @ %d Hz%s",
                    spi_device.c_str(), render_hz, headless ? " (headless)" : "");
    }

    ~EyesNode() override {
        if (_dev) {
            try {
                _dev->off();
            } catch (const std::exception&) {
            }
        }
    }

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
        inputs.gesture = _gesture;
        inputs.x = _body_pose.x;
        inputs.y = _body_pose.y;
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
        // animation is active — then its own steps own the gaze.
        _target.expr = _last_target.expression;
        if (animation == nullptr) {
            _target.gaze = _last_target.gaze;
            _target.gazeEaseMs = 0;
            _target.gazeScaleX = _last_target.gaze_scale_x;
            _target.gazeScaleY = _last_target.gaze_scale_y;
        } else {
            _runner.run(*animation, now);
            _target.gazeEaseMs =
                static_cast<uint32_t>(animation->gaze_ease_s() * 1000.0);
            _target.gazeScaleX = static_cast<float>(animation->gaze_scale());
            _target.gazeScaleY = _target.gazeScaleX;
        }
    }

    void show(const Image& frame) {
        if (!_dev) return;
        try {
            _dev->show(frame);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "panel write failed: %s",
                                  e.what());
        }
    }

    void renderTick() {
        if (!_display_text.empty()) {
            // Text mode: the message is static, so redraw only on change.
            if (_display_text != _drawn_text) {
                _drawn_text = _display_text;
                _text_panel.clearBuffer();
                hexa::display::drawTextScreen(_text_panel.u8g2(), _display_text, _text_cfg);
                show(hexa::eyes_cpp::EyesFace::fromU8g2(_text_panel.u8g2()));
            }
            return;
        }
        _drawn_text.clear();

        RenderState screen = _target;
        screen.gaze = hexa::display::toScreenGaze(screen.gaze);
        const uint32_t now = nowMs();
        const eyes::AnimFrame f = _anim.update(screen, now);
        show(_face->render(f, now / 1000.0));
    }

    // Render half.
    std::unique_ptr<Sh1122> _dev;          // null when headless
    face::Sh1122Panel _text_panel;         // headless u8g2 buffer for text mode
    std::unique_ptr<hexa::eyes_cpp::EyesFace> _face;
    eyes::EyeAnim _anim;
    std::mt19937 _rng;
    RenderState _target{Expression::NEUTRAL, GazeDirection::CENTER};
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
    std::string _gesture;
    geometry_msgs::msg::Twist _cmd_vel;
    hexa_interfaces::msg::BodyPose _body_pose;
    std::string _animation_mode;
    std::optional<double> _battery_voltage;
    std::string _display_text;
    bool _bluetooth_scanning = false;
    bool _display_busy = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_gait;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_gesture;
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
        rclcpp::spin(std::make_shared<EyesNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("eyes_node"), "fatal: %s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}

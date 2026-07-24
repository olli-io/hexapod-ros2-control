#!/bin/sh
# Boot jingle for a passive buzzer on the Pi's hardware PWM. Run at boot by
# systemd/hexa-boot-tune.service, or by hand via `./hexa robot play-tune`.
#
# Dependency-free by design: POSIX sh + the kernel's sysfs PWM interface, no
# Python, no gpiozero, no packages to install. It runs long before Docker, the
# container, or the servo rail exist, so it is the earliest "the Pi is alive"
# signal the robot has.
#
# Wiring (Raspberry Pi 5): buzzer + -> GPIO18 (BCM), buzzer - -> GND, ~100 ohm
# in series if it is too loud. GPIO18 is RP1 PWM0 channel 2, alt function a3.
# See docs/robot-environment.md for the config.txt overlay.
#
# Everything is overridable from the environment so a different buzzer pin or
# tune needs no edit here:
#   TUNE_GPIO       BCM pin the buzzer is on            (default 18)
#   TUNE_CHANNEL    PWM channel that pin maps to        (default 2)
#   TUNE_PIN_ALT    pinctrl alt function; "" to skip    (default a3)
#   TUNE_PWMCHIP    force a /sys/class/pwm/pwmchipN     (default: discovered)
#   TUNE_TEMPO      seconds per beat                    (default 0.150)
#   TUNE_MELODY     "NOTE:beats ...", REST for silence
#
# Failure is never fatal: no buzzer fitted, no PWM overlay, or a busy channel
# all log a line and exit 0. A boot tune must not be able to hold up a boot.

GPIO="${TUNE_GPIO:-18}"
CHANNEL="${TUNE_CHANNEL:-2}"
PIN_ALT="${TUNE_PIN_ALT-a3}"
TEMPO="${TUNE_TEMPO:-0.150}"

# A short, unmistakable two-note "I'm up". The notes need real duration and a
# rest between them, or a very short first note runs straight into the second
# and the two blur into one continuous rising beep.
MELODY="${TUNE_MELODY:-B5:2 REST:1 E6:4}"

# How long to wait for the PWM sysfs tree to appear. The RP1 PWM driver probes
# early, but the unit can still win the race on a cold boot.
WAIT_S=5

PWM=""          # exported channel dir, e.g. /sys/class/pwm/pwmchip2/pwm2
CHIP=""

log() { echo "boot-tune: $*" >&2; }

# Equal temperament, A4 = 440 Hz, rounded to whole Hz. Passive buzzers are
# happiest in the C5-C7 range; below C4 most of them are barely audible.
note_hz() {
    case "$1" in
        C4)  echo 262 ;; C#4|Db4) echo 277 ;; D4) echo 294 ;; D#4|Eb4) echo 311 ;;
        E4)  echo 330 ;; F4)  echo 349 ;; F#4|Gb4) echo 370 ;; G4) echo 392 ;;
        G#4|Ab4) echo 415 ;; A4) echo 440 ;; A#4|Bb4) echo 466 ;; B4) echo 494 ;;
        C5)  echo 523 ;; C#5|Db5) echo 554 ;; D5) echo 587 ;; D#5|Eb5) echo 622 ;;
        E5)  echo 659 ;; F5)  echo 698 ;; F#5|Gb5) echo 740 ;; G5) echo 784 ;;
        G#5|Ab5) echo 831 ;; A5) echo 880 ;; A#5|Bb5) echo 932 ;; B5) echo 988 ;;
        C6)  echo 1047 ;; C#6|Db6) echo 1109 ;; D6) echo 1175 ;; D#6|Eb6) echo 1245 ;;
        E6)  echo 1319 ;; F6)  echo 1397 ;; F#6|Gb6) echo 1480 ;; G6) echo 1568 ;;
        G#6|Ab6) echo 1661 ;; A6) echo 1760 ;; A#6|Bb6) echo 1865 ;; B6) echo 1976 ;;
        C7)  echo 2093 ;; D7) echo 2349 ;; E7) echo 2637 ;; G7) echo 3136 ;;
        *)   return 1 ;;
    esac
}

# Pick the pwmchip that owns the buzzer pin. On a Pi 5 the SoC's 2-channel block
# probes first as pwmchip0 and RP1's 4-channel block lands on pwmchip2 — but the
# numbering shifts with the kernel and the overlays in play, so discover it
# rather than hardcoding. RP1 is the only chip with 4 channels, which is the
# reliable discriminator; the sysfs device path is checked first when it names
# rp1 outright.
find_chip() {
    if [ -n "${TUNE_PWMCHIP:-}" ]; then
        echo "${TUNE_PWMCHIP}"
        return 0
    fi

    _best=""
    for _c in /sys/class/pwm/pwmchip*; do
        [ -r "${_c}/npwm" ] || continue
        _n="$(cat "${_c}/npwm" 2>/dev/null)" || continue
        # Must at least have the channel we want.
        [ "${_n}" -gt "${CHANNEL}" ] 2>/dev/null || continue

        case "$(readlink -f "${_c}/device" 2>/dev/null)" in
            *rp1*|*RP1*) echo "${_c}"; return 0 ;;
        esac
        # 4 channels = RP1's block; prefer it over the SoC's 2-channel one.
        if [ "${_n}" -ge 4 ]; then
            echo "${_c}"
            return 0
        fi
        [ -z "${_best}" ] && _best="${_c}"
    done

    [ -n "${_best}" ] || return 1
    echo "${_best}"
}

# Always leave the buzzer silent and the channel released, however we exit —
# a killed script must not leave the thing screaming.
cleanup() {
    if [ -n "${PWM}" ] && [ -d "${PWM}" ]; then
        echo 0 > "${PWM}/enable" 2>/dev/null
    fi
    if [ -n "${CHIP}" ] && [ -w "${CHIP}/unexport" ]; then
        echo "${CHANNEL}" > "${CHIP}/unexport" 2>/dev/null
    fi
}

# One note. hz=0 is a rest: stay disabled for the duration.
tone() {
    _hz="$1"
    _secs="$2"

    if [ "${_hz}" -gt 0 ] 2>/dev/null; then
        _period=$(( 1000000000 / _hz ))
        # Disable before re-periodising: the kernel rejects a duty_cycle that
        # exceeds the period still in force, so order matters here.
        echo 0 > "${PWM}/enable" 2>/dev/null
        echo "${_period}" > "${PWM}/period" 2>/dev/null || {
            log "cannot set period ${_period}ns (${_hz} Hz) — skipping note"
            return 0
        }
        # 50% duty = square wave, the loudest a passive buzzer gets.
        echo $(( _period / 2 )) > "${PWM}/duty_cycle" 2>/dev/null
        echo 1 > "${PWM}/enable" 2>/dev/null
    fi

    sleep "${_secs}"
    echo 0 > "${PWM}/enable" 2>/dev/null
}

# --- Wait for the PWM tree, then claim the channel -------------------------

_waited=0
while [ ! -d /sys/class/pwm ] || ! CHIP="$(find_chip)"; do
    [ "${_waited}" -ge "${WAIT_S}" ] && break
    sleep 1
    _waited=$(( _waited + 1 ))
done

if [ -z "${CHIP}" ] || [ ! -d "${CHIP}" ]; then
    log "no usable pwmchip with channel ${CHANNEL} — is the PWM overlay in /boot/firmware/config.txt? Staying silent."
    exit 0
fi

# The stock overlay puts GPIO12/13/18/19 into their PWM alt modes at boot, but
# that depends on which overlay is in config.txt. Re-assert it; pinctrl ships
# with Pi OS (raspi-utils), and its absence is not an error.
if [ -n "${PIN_ALT}" ] && command -v pinctrl >/dev/null 2>&1; then
    pinctrl set "${GPIO}" "${PIN_ALT}" 2>/dev/null \
        || log "pinctrl could not set GPIO${GPIO} to ${PIN_ALT} (continuing)"
fi

trap cleanup EXIT INT TERM HUP

if [ ! -d "${CHIP}/pwm${CHANNEL}" ]; then
    echo "${CHANNEL}" > "${CHIP}/export" 2>/dev/null || {
        log "cannot export channel ${CHANNEL} on ${CHIP} (need root? already in use?) — staying silent."
        exit 0
    }
    # udev applies the channel dir's permissions asynchronously.
    _t=0
    while [ ! -d "${CHIP}/pwm${CHANNEL}" ] && [ "${_t}" -lt 20 ]; do
        sleep 0.1
        _t=$(( _t + 1 ))
    done
fi

PWM="${CHIP}/pwm${CHANNEL}"
if [ ! -d "${PWM}" ]; then
    log "channel dir ${PWM} never appeared — staying silent."
    exit 0
fi

# --- Play ------------------------------------------------------------------

for token in ${MELODY}; do
    note="${token%%:*}"
    beats="${token##*:}"
    [ "${note}" = "${token}" ] && beats=1     # bare note = one beat

    if [ "${note}" = "REST" ] || [ "${note}" = "-" ]; then
        hz=0
    elif ! hz="$(note_hz "${note}")"; then
        log "unknown note '${note}' — skipping"
        continue
    fi

    # The only float arithmetic in the script. awk is POSIX-mandated and
    # present on every Pi OS image, so this costs no installed dependency.
    secs="$(awk -v b="${beats}" -v t="${TEMPO}" 'BEGIN { printf "%.4f", b * t }')"
    tone "${hz}" "${secs}"
done

exit 0

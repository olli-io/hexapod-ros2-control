#!/bin/sh
# Tune player for the passive buzzer on the Pi's hardware PWM. The robot's only
# audible channel: "the Pi is alive", "the stack is up", "I tripped", and "you
# can cut the power now".
#
#   buzzer.sh [tune]           play a named tune (default: boot)
#   buzzer.sh --spool FILE     play the tune named in FILE's first word
#
# Wiring: buzzer + -> GPIO12 (BCM), buzzer - -> GND, ~100 ohm in series if it is
# too loud. Not GPIO18, the pin a bare `dtoverlay=pwm-2chan` takes: that line is
# I2S PCM_CLK and SPI1 CE0, worth leaving free. See docs/robot-environment.md
# for the config.txt overlay that muxes GPIO12 to PWM.
#
# POSIX sh + sysfs only, so it needs nothing installed on the host. It always
# runs on the *host*, never in the container: Docker mounts /sys read-only, so
# an export from inside fails with EROFS. Container-borne beeps (up, fault,
# undervolt) arrive via --spool. See docs/robot-environment.md.
#
# Overridable: TUNE_GPIO, TUNE_PWMCHIP, TUNE_CHANNEL, TUNE_TEMPO, TUNE_WAIT_S,
# TUNE_MELODY. TUNE_GPIO is only ever a label — config.txt owns the pin mux.
#
# Failure is never fatal: no buzzer fitted, no overlay, or a busy channel all
# log a line and exit 0. A beep must not hold up a boot, a shutdown, or the
# fault path that asked for it.

# RP1's PWM0 block on a Pi 5; GPIO12 is its channel 0, alt function a0. On a Pi
# 4 set TUNE_PWMCHIP=/sys/class/pwm/pwmchip0 (channel 0 is the same there).
PWM_DEV="/sys/bus/platform/devices/1f00098000.pwm/pwm"
CHANNEL="${TUNE_CHANNEL:-0}"
GPIO="${TUNE_GPIO:-12}"     # log line only; the overlay does the muxing

TEMPO="${TUNE_TEMPO:-0.150}"
# 0 for anything playing after boot; only a cold boot can outrun the PWM driver.
WAIT_S="${TUNE_WAIT_S:-5}"

PWM=""          # exported channel dir, e.g. .../pwm/pwmchip0/pwm2
CHIP=""

log() { echo "buzzer: $*" >&2; }

# Each tune has to be recognisable through a closing door, so they differ in
# contour, not just pitch: rising = good, falling = done, alternating = wrong.
# Notes need a rest between them or they blur into one continuous beep.
tune_melody() {
    case "$1" in
        boot)      echo "B5:2 REST:1 E6:4" ;;              # kernel is up
        up)        echo "E6:2 REST:1 G#6:2 REST:1 B6:4" ;; # stack is live
        shutdown)  echo "E6:2 REST:1 B5:4" ;;              # safe to cut power
        fault)     echo "C6:2 G5:2 C6:2 G5:2 C6:2 G5:4" ;; # over-current klaxon
        undervolt) echo "G5:12" ;;                         # pack low, still drivable
        *)         return 1 ;;
    esac
}

# --- Pick the tune ---------------------------------------------------------

TUNE="boot"
case "${1:-}" in
    "") ;;
    --spool)
        [ -n "${2:-}" ] || { log "--spool needs a file argument"; exit 0; }
        # Read-only by design: writing back to the file the .path unit watches
        # would retrigger it forever.
        _req="$(head -n 1 "$2" 2>/dev/null | tr -d '[:space:]')"
        [ -n "${_req}" ] || { log "spool $2 is empty or unreadable — staying silent."; exit 0; }
        TUNE="${_req}"
        ;;
    -h|--help)
        echo "usage: buzzer.sh [boot|up|shutdown|fault|undervolt] | buzzer.sh --spool FILE"
        exit 0
        ;;
    -*) log "unknown option '$1' — staying silent."; exit 0 ;;
    *)  TUNE="$1" ;;
esac

if [ -n "${TUNE_MELODY:-}" ]; then
    MELODY="${TUNE_MELODY}"
    TUNE="custom"
elif ! MELODY="$(tune_melody "${TUNE}")"; then
    log "unknown tune '${TUNE}' — staying silent."
    exit 0
fi

# Equal temperament, A4 = 440 Hz. Passive buzzers are happiest in C5-C7.
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

# config.txt fixes the pin mux but not the sysfs chip number, which is kernel
# probe order and has moved between releases. The platform address is fixed, so
# reach the block through it; the glob resolves the single chip underneath.
find_chip() {
    if [ -n "${TUNE_PWMCHIP:-}" ]; then
        echo "${TUNE_PWMCHIP}"
        return 0
    fi

    for _c in "${PWM_DEV}"/pwmchip*; do
        [ -d "${_c}" ] && { echo "${_c}"; return 0; }
    done
    return 1
}

# Always leave the buzzer silent and the channel released, however we exit.
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
        # Disable first: the kernel rejects a duty_cycle exceeding the period
        # still in force, so order matters here.
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

# --- Claim the channel -----------------------------------------------------

_waited=0
while ! CHIP="$(find_chip)"; do
    [ "${_waited}" -ge "${WAIT_S}" ] && break
    sleep 1
    _waited=$(( _waited + 1 ))
done

if [ -z "${CHIP}" ] || [ ! -d "${CHIP}" ]; then
    log "no pwmchip at ${CHIP:-${PWM_DEV}} — is dtoverlay=pwm-2chan in /boot/firmware/config.txt? Staying silent."
    exit 0
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

log "playing '${TUNE}' on GPIO${GPIO} (${CHIP##*/} channel ${CHANNEL})"

for token in ${MELODY}; do
    note="${token%%:*}"
    beats="${token##*:}"
    [ "${note}" = "${token}" ] && beats=1     # bare note = one beat

    if [ "${note}" = "REST" ]; then
        hz=0
    elif ! hz="$(note_hz "${note}")"; then
        log "unknown note '${note}' — skipping"
        continue
    fi

    # awk is POSIX-mandated, so this float multiply costs no dependency.
    secs="$(awk -v b="${beats}" -v t="${TEMPO}" 'BEGIN { printf "%.4f", b * t }')"
    tone "${hz}" "${secs}"
done

exit 0

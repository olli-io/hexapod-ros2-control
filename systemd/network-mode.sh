#!/bin/sh
# Switches the Pi between joining a wifi network ("station") and hosting the
# `hexapod` access point that serves the web teleop ("hotspot").
#
#   network-mode.sh [hotspot|station|toggle|status]
#   network-mode.sh --spool FILE        act on a request written by the container
#   network-mode.sh --report            write the current mode, change nothing
#   network-mode.sh --install-portal    write the captive-DNS drop-in
#   network-mode.sh --uninstall-portal  remove it, and any leftover redirect
#
# Options: --state FILE names where to report to (default: none, print only).
#
# It always runs on the *host*, never in the container, for the same reason
# buzzer.sh does. The ROS stack runs unprivileged with host networking, no
# D-Bus socket and no NET_ADMIN, so it cannot talk to NetworkManager at all.
# hexa_buttons writes a request into the bind-mounted log volume and
# hexa-network-spool.path runs this script out here. See
# docs/robot-environment.md and src/hexa_buttons/hexa_buttons/network_state.py.
#
# NetworkManager does the heavy lifting, so there is no hostapd and no dnsmasq
# package to install: an AP profile with ipv4.method=shared makes NM run its own
# dnsmasq for DHCP and DNS on that interface. Requires Pi OS Bookworm or newer
# (NetworkManager-based) with wlan0 managed, and a wifi country set — an AP will
# not start without one (`raspi-config`, Localisation -> WLAN Country).
#
# The web teleop needs no reverse proxy: it is already an aiohttp server bound
# to 0.0.0.0:8080, and with host networking that includes the AP interface. All
# this adds is a port-80 redirect so any address a phone types lands on it.
#
# Everything is overridable from the environment:
#   HEXA_AP_SSID      network name to advertise          (default hexapod)
#   HEXA_AP_PSK       WPA2 passphrase, 8-63 chars        (default hexahexa)
#   HEXA_AP_CON       NetworkManager profile name        (default hexapod-ap)
#   HEXA_AP_IFACE     wireless interface                 (default wlan0)
#   HEXA_AP_ADDR      the robot's address on the AP      (default 192.168.4.1)
#   HEXA_AP_CIDR      prefix length for it               (default 24)
#   HEXA_AP_CHANNEL   2.4 GHz channel                    (default 6)
#   HEXA_PORTAL_PORT  port the web teleop listens on     (default 8080)
#   HEXA_NM_WAIT      seconds to allow nmcli per step    (default 45)
#
# Failure is never fatal: every path exits 0 and reports a short reason instead,
# which the panel turns into a sentence (info_text.network_error_reason). A
# network switch must not be able to wedge a systemd unit or hold up a boot.

set -u

SSID="${HEXA_AP_SSID:-hexapod}"
PSK="${HEXA_AP_PSK:-hexahexa}"
CON="${HEXA_AP_CON:-hexapod-ap}"
IFACE="${HEXA_AP_IFACE:-wlan0}"
ADDR="${HEXA_AP_ADDR:-192.168.4.1}"
CIDR="${HEXA_AP_CIDR:-24}"
CHANNEL="${HEXA_AP_CHANNEL:-6}"
PORTAL_PORT="${HEXA_PORTAL_PORT:-8080}"
NM_WAIT="${HEXA_NM_WAIT:-45}"

# Where the previously-active station profile is remembered, so a hotspot that
# fails to come up can put the robot back on the network it was already on.
# Deliberately NOT in the log volume: that directory is watched by the .path
# unit, and a write there would retrigger this script.
VAR_DIR=/var/lib/hexa-network
PREV_FILE="${VAR_DIR}/previous-profile"

PORTAL_DNS=/etc/NetworkManager/dnsmasq-shared.d/hexa-captive.conf
NFT_TABLE=hexa_portal

STATE_FILE=""
TOKEN=""

log() { echo "network-mode: $*" >&2; }

# --- Reporting -------------------------------------------------------------

# write_state MODE RESULT [REASON]
#
# tmp + rename, so the container can never read a half-written line: it polls
# this file's mtime from a 20 Hz tick and parses whatever it finds. (The request
# file is the opposite — written in place, never renamed — because systemd's
# PathModified watches that inode and a rename would break the watch.)
write_state() {
    [ -n "${STATE_FILE}" ] || return 0
    _tmp="${STATE_FILE}.tmp"
    {
        echo "# written by network-mode.sh — do not edit"
        echo "mode=$1"
        echo "token=${TOKEN}"
        echo "result=$2"
        echo "reason=${3:-}"
        echo "ssid=${SSID}"
        echo "psk=${PSK}"
    } > "${_tmp}" 2>/dev/null || { log "cannot write ${_tmp}"; return 0; }
    chmod 644 "${_tmp}" 2>/dev/null
    mv -f "${_tmp}" "${STATE_FILE}" 2>/dev/null || log "cannot replace ${STATE_FILE}"
}

# fail REASON — report and stop, without changing the mode we believe we are in.
fail() {
    log "$1"
    write_state "$(current_mode)" error "$1"
    exit 0
}

# --- Inspection ------------------------------------------------------------

# The host is the authority on which mode the radio is in: the AP profile being
# active is the whole definition, so there is no state to keep in sync.
current_mode() {
    if nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "${CON}"; then
        echo hotspot
    else
        echo station
    fi
}

current_ip() {
    ip -4 -o addr show dev "${IFACE}" 2>/dev/null |
        awk '{ split($4, a, "/"); print a[1]; exit }'
}

# The station profile currently up on our interface, if any.
active_station_profile() {
    nmcli -t -f NAME,DEVICE,TYPE connection show --active 2>/dev/null |
        awk -F: -v i="${IFACE}" -v c="${CON}" \
            '$2 == i && $3 == "802-11-wireless" && $1 != c { print $1; exit }'
}

preflight() {
    command -v nmcli >/dev/null 2>&1 || fail nmcli-missing
    _state="$(nmcli -t -f DEVICE,STATE device 2>/dev/null |
        awk -F: -v i="${IFACE}" '$1 == i { print $2; exit }')"
    [ -n "${_state}" ] || fail no-wifi
    [ "${_state}" = "unmanaged" ] && fail unmanaged
    # An AP will not start without a regulatory domain, and the failure from
    # nmcli is opaque, so name it up front.
    if command -v iw >/dev/null 2>&1; then
        if iw reg get 2>/dev/null | grep -q 'country 00:'; then
            fail no-country
        fi
    fi
    return 0
}

# --- Captive portal --------------------------------------------------------

# Wildcard DNS, so any hostname a phone asks for resolves to the robot. NM only
# starts a dnsmasq for `shared` connections, so this drop-in is inert while the
# Pi is in station mode — which is why it is installed once rather than toggled.
portal_dns_install() {
    _dir="$(dirname "${PORTAL_DNS}")"
    mkdir -p "${_dir}" 2>/dev/null || { log "cannot create ${_dir}"; return 1; }
    cat > "${PORTAL_DNS}" <<EOF || { log "cannot write ${PORTAL_DNS}"; return 1; }
# Captive portal for the hexapod access point. Generated by network-mode.sh.
#
# NetworkManager's ipv4.method=shared runs its own dnsmasq for the AP; this
# makes it answer every name with the robot, so a phone that types anything at
# all lands on the web teleop. Only ever read while the AP is up.
address=/#/${ADDR}
# No upstream to forward to, and no point leaking queries at one.
no-resolv
EOF
    log "captive DNS drop-in installed at ${PORTAL_DNS}"
}

portal_dns_remove() {
    rm -f "${PORTAL_DNS}" 2>/dev/null
    log "captive DNS drop-in removed"
}

# Port 80 -> the teleop server. Its own nftables table, so teardown is one
# command and we can never damage a rule somebody else owns. This also catches
# clients that ignore DHCP's DNS and hard-code a resolver, which the wildcard
# above cannot reach.
portal_redirect_up() {
    command -v nft >/dev/null 2>&1 || { log "no nft — skipping the port 80 redirect"; return 0; }
    portal_redirect_down
    nft add table ip "${NFT_TABLE}" 2>/dev/null || return 0
    nft add chain ip "${NFT_TABLE}" prerouting \
        '{ type nat hook prerouting priority dstnat; policy accept; }' 2>/dev/null || return 0
    nft add rule ip "${NFT_TABLE}" prerouting iifname "${IFACE}" \
        tcp dport 80 redirect to :"${PORTAL_PORT}" 2>/dev/null ||
        log "could not add the port 80 redirect"
}

portal_redirect_down() {
    command -v nft >/dev/null 2>&1 || return 0
    nft delete table ip "${NFT_TABLE}" 2>/dev/null || true
}

# --- Switching -------------------------------------------------------------

ensure_ap_profile() {
    if ! nmcli -t -f NAME connection show 2>/dev/null | grep -qx "${CON}"; then
        nmcli connection add type wifi ifname "${IFACE}" con-name "${CON}" \
            autoconnect no ssid "${SSID}" >/dev/null 2>&1 ||
            fail ap-failed
    fi
    # Re-applied every time, so changing HEXA_AP_SSID/PSK in the unit takes
    # effect without anyone having to delete a stale profile by hand.
    #
    # autoconnect no is load-bearing: a robot that reboots must come back on the
    # home network, reachable from the workstation, not stuck hosting an AP.
    nmcli connection modify "${CON}" \
        connection.autoconnect no \
        802-11-wireless.ssid "${SSID}" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        802-11-wireless.channel "${CHANNEL}" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.proto rsn \
        wifi-sec.pairwise ccmp \
        wifi-sec.group ccmp \
        wifi-sec.psk "${PSK}" \
        ipv4.method shared \
        ipv4.addresses "${ADDR}/${CIDR}" \
        ipv6.method disabled >/dev/null 2>&1 || fail ap-failed
}

go_hotspot() {
    log "switching to hotspot '${SSID}' on ${IFACE}"
    # Remember where we came from BEFORE bringing the AP up, so a failure can
    # put the robot back rather than leaving it with no network at all. Guarded
    # rather than best-effort-redirected: a failed redirection is the shell's
    # own error, not the command's, so 2>/dev/null would not silence it.
    if mkdir -p "${VAR_DIR}" 2>/dev/null; then
        active_station_profile > "${PREV_FILE}" 2>/dev/null || true
    else
        log "cannot create ${VAR_DIR} — rollback will fall back to autoconnect"
    fi

    rfkill unblock wlan 2>/dev/null || true
    ensure_ap_profile
    portal_dns_install || true

    if ! nmcli -w "${NM_WAIT}" connection up "${CON}" >/dev/null 2>&1; then
        log "the access point would not start — rolling back"
        _prev="$(cat "${PREV_FILE}" 2>/dev/null)"
        if [ -n "${_prev}" ]; then
            nmcli -w "${NM_WAIT}" connection up "${_prev}" >/dev/null 2>&1 || true
        else
            nmcli -w "${NM_WAIT}" device connect "${IFACE}" >/dev/null 2>&1 || true
        fi
        write_state station error ap-failed
        exit 0
    fi

    portal_redirect_up
    log "hotspot up: join '${SSID}' and browse to http://${ADDR}/"
    write_state hotspot ok
}

go_station() {
    log "switching back to the wifi network"
    portal_redirect_down
    nmcli connection down "${CON}" >/dev/null 2>&1 || true

    # NetworkManager will usually autoconnect on its own once the AP releases
    # the radio; naming the previous profile just makes it immediate.
    _prev="$(cat "${PREV_FILE}" 2>/dev/null)"
    if [ -n "${_prev}" ] &&
        nmcli -w "${NM_WAIT}" connection up "${_prev}" >/dev/null 2>&1; then
        :
    elif nmcli -w "${NM_WAIT}" device connect "${IFACE}" >/dev/null 2>&1; then
        :
    else
        # Not fatal to the radio: NM keeps retrying its autoconnect profiles, so
        # this often comes good on its own a few seconds later.
        write_state station error station-failed
        exit 0
    fi

    log "back on the wifi network as $(current_ip)"
    write_state station ok
}

# --- Entry point -----------------------------------------------------------

ACTION=""
while [ $# -gt 0 ]; do
    case "$1" in
        --state)
            [ -n "${2:-}" ] || { log "--state needs a file argument"; exit 0; }
            STATE_FILE="$2"
            shift 2
            ;;
        --spool)
            [ -n "${2:-}" ] || { log "--spool needs a file argument"; exit 0; }
            # First line only, and read-only: writing this file back would be
            # another modification of the very file the .path unit watches, and
            # the trigger would loop forever. Same rule as the buzzer spool.
            _line="$(head -n 1 "$2" 2>/dev/null)"
            ACTION="$(echo "${_line}" | awk '{ print $1 }')"
            TOKEN="$(echo "${_line}" | awk '{ print $2 }')"
            [ -n "${ACTION}" ] || { log "spool $2 is empty or unreadable"; exit 0; }
            shift 2
            ;;
        --report) ACTION=report; shift ;;
        --install-portal) ACTION=install-portal; shift ;;
        --uninstall-portal) ACTION=uninstall-portal; shift ;;
        -h|--help)
            echo "usage: network-mode.sh [hotspot|station|toggle|status] [--state FILE]"
            echo "       network-mode.sh --spool FILE [--state FILE]"
            echo "       network-mode.sh --report [--state FILE]"
            echo "       network-mode.sh --install-portal | --uninstall-portal"
            exit 0
            ;;
        -*) log "unknown option '$1'"; exit 0 ;;
        *)  ACTION="$1"; shift ;;
    esac
done

case "${ACTION:-status}" in
    install-portal)
        portal_dns_install
        exit 0
        ;;
    uninstall-portal)
        portal_dns_remove
        portal_redirect_down
        exit 0
        ;;
esac

command -v nmcli >/dev/null 2>&1 || fail nmcli-missing

case "${ACTION:-status}" in
    status|report)
        _mode="$(current_mode)"
        log "mode=${_mode} address=$(current_ip)"
        write_state "${_mode}" ok
        ;;
    hotspot|station|toggle)
        # Acknowledge before the slow part. The container waits only a few
        # seconds for this: silence means the units were never installed, and it
        # says so on the panel instead of spinning for the full timeout.
        write_state "$(current_mode)" switching
        preflight
        _target="${ACTION}"
        if [ "${_target}" = "toggle" ]; then
            # The host owns the radio, so it is also the only side that reliably
            # knows which way round it currently is — which is why the button
            # sends `toggle` rather than naming a target.
            if [ "$(current_mode)" = "hotspot" ]; then
                _target=station
            else
                _target=hotspot
            fi
        fi
        if [ "${_target}" = "hotspot" ]; then
            go_hotspot
        else
            go_station
        fi
        ;;
    *)
        log "unknown action '${ACTION}'"
        write_state "$(current_mode)" error "unknown-action"
        ;;
esac

exit 0

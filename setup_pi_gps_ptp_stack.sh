#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IFACE="eth0"
PPS_GPIO="18"
I2C_BUS="1"
GPS_ADDR="0x42"
CHRONY_SOCK_PATH="/run/chrony/zedf9p.sock"
BRIDGE_FIX_JSON_PATH="/run/ublox_i2c_chrony_bridge/latest_fix.json"
BRIDGE_INSTALL_PATH="/usr/local/bin/ublox_i2c_chrony_bridge.py"
BRIDGE_SERVICE_PATH="/etc/systemd/system/ublox-i2c-chrony-bridge.service"
PTP4L_SERVICE_PATH="/etc/systemd/system/ptp4l.service"
PHC2SYS_SERVICE_PATH="/etc/systemd/system/phc2sys.service"
CHRONY_CONF_DIR="/etc/chrony/conf.d"
CHRONY_MAIN_CONF="/etc/chrony/chrony.conf"
CHRONY_REFCLK_CONF="${CHRONY_CONF_DIR}/zedf9p-gps-pps.conf"
PTP4L_CONF="/etc/linuxptp/ptp4l-zedf9p-master.conf"
APT_PACKAGES=(
  chrony
  curl
  ethtool
  i2c-tools
  linuxptp
  pps-tools
  python3-smbus
)

usage() {
  cat <<'EOF'
Usage:
  sudo ./setup_pi_gps_ptp_stack.sh [options]

Options:
  --iface IFACE           Ethernet interface for PTP grandmaster (default: eth0)
  --pps-gpio GPIO         BCM GPIO number for PPS input (default: 18; physical pin 12)
  --i2c-bus BUS           I2C bus number (default: 1)
  --gps-addr ADDR         GPS I2C address (default: 0x42)
  --chrony-sock PATH      chrony SOCK refclock path (default: /run/chrony/zedf9p.sock)
  --bridge-fix-json PATH  JSON path for latest published GPS fix (default: /run/ublox_i2c_chrony_bridge/latest_fix.json)
  --skip-apt              Skip apt package installation
  --no-start              Enable services but do not start/restart them now
  -h, --help              Show this help

Notes:
  - This script configures a ZED-F9P connected via I2C + PPS.
  - It installs a custom I2C->chrony SOCK bridge because gpsd is not a drop-in
    solution for a GPS connected only over I2C.
  - A reboot is required if the script updates boot config for I2C/PPS overlays.
EOF
}

SKIP_APT="0"
NO_START="0"
BOOT_CONFIG_CHANGED="0"

log() {
  printf '[INFO] %s\n' "$1"
}

warn() {
  printf '[WARN] %s\n' "$1"
}

require_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "Run this script with sudo/root." >&2
    exit 1
  fi
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --iface)
        IFACE="$2"
        shift 2
        ;;
      --pps-gpio)
        PPS_GPIO="$2"
        shift 2
        ;;
      --i2c-bus)
        I2C_BUS="$2"
        shift 2
        ;;
      --gps-addr)
        GPS_ADDR="$2"
        shift 2
        ;;
      --chrony-sock)
        CHRONY_SOCK_PATH="$2"
        shift 2
        ;;
      --bridge-fix-json)
        BRIDGE_FIX_JSON_PATH="$2"
        shift 2
        ;;
      --skip-apt)
        SKIP_APT="1"
        shift
        ;;
      --no-start)
        NO_START="1"
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "Unknown option: $1" >&2
        usage
        exit 2
        ;;
    esac
  done
}

detect_boot_config() {
  if [[ -f /boot/firmware/config.txt ]]; then
    echo /boot/firmware/config.txt
  elif [[ -f /boot/config.txt ]]; then
    echo /boot/config.txt
  else
    echo ""
  fi
}

ensure_line_in_file() {
  local file_path="$1"
  local line="$2"

  if grep -Fxq "${line}" "${file_path}"; then
    return
  fi

  printf '\n%s\n' "${line}" >>"${file_path}"
  BOOT_CONFIG_CHANGED="1"
}

install_packages() {
  if [[ "${SKIP_APT}" == "1" ]]; then
    warn "Skipping apt package installation as requested"
    return
  fi

  log "Installing required packages"
  apt-get update
  apt-get install -y "${APT_PACKAGES[@]}"
}

configure_boot() {
  local boot_config
  boot_config="$(detect_boot_config)"
  if [[ -z "${boot_config}" ]]; then
    warn "Could not find /boot/firmware/config.txt or /boot/config.txt. Skipping boot config changes."
    return
  fi

  log "Ensuring I2C and PPS overlay in ${boot_config}"
  ensure_line_in_file "${boot_config}" "dtparam=i2c_arm=on"

  if grep -Eq '^[[:space:]]*dtoverlay=pps-gpio,' "${boot_config}"; then
    if grep -Eq "^[[:space:]]*dtoverlay=pps-gpio,gpiopin=${PPS_GPIO}([[:space:]]*|,.*)$" "${boot_config}"; then
      :
    else
      warn "A pps-gpio overlay already exists in ${boot_config}. Review it manually if the GPIO differs from ${PPS_GPIO}."
    fi
  else
    ensure_line_in_file "${boot_config}" "dtoverlay=pps-gpio,gpiopin=${PPS_GPIO}"
  fi
}

install_bridge_script() {
  log "Installing I2C->chrony bridge to ${BRIDGE_INSTALL_PATH}"
  install -m 0755 "${SCRIPT_DIR}/ublox_i2c_chrony_bridge.py" "${BRIDGE_INSTALL_PATH}"
}

write_bridge_service() {
  log "Writing ${BRIDGE_SERVICE_PATH}"
  cat >"${BRIDGE_SERVICE_PATH}" <<EOF
[Unit]
Description=u-blox I2C to chrony SOCK bridge
After=chrony.service network-online.target
Wants=chrony.service network-online.target

[Service]
Type=simple
ExecStart=${BRIDGE_INSTALL_PATH} --i2c-bus ${I2C_BUS} --i2c-addr ${GPS_ADDR} --sock-path ${CHRONY_SOCK_PATH} --fix-json-path ${BRIDGE_FIX_JSON_PATH}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
}

configure_chrony() {
  log "Configuring chrony refclocks"
  mkdir -p "${CHRONY_CONF_DIR}"

  if [[ -f "${CHRONY_MAIN_CONF}" ]]; then
    if ! grep -Eq '^[[:space:]]*confdir[[:space:]]+/etc/chrony/conf\.d' "${CHRONY_MAIN_CONF}"; then
      printf '\nconfdir /etc/chrony/conf.d\n' >>"${CHRONY_MAIN_CONF}"
    fi
  else
    warn "${CHRONY_MAIN_CONF} not found. Chrony package may not be installed yet."
  fi

  cat >"${CHRONY_REFCLK_CONF}" <<EOF
# ZED-F9P I2C + PPS time sources
# GPS SOCK source provides coarse UTC date/time from the custom I2C bridge.
# PPS provides precise second boundaries and is locked to the GPS source.
refclock SOCK ${CHRONY_SOCK_PATH} refid GPS noselect poll 2 filter 8
refclock PPS /dev/pps0 lock GPS refid PPS prefer poll 2
EOF
}

configure_ptp4l() {
  log "Writing ${PTP4L_CONF}"
  mkdir -p "$(dirname "${PTP4L_CONF}")"
  cat >"${PTP4L_CONF}" <<EOF
[global]
twoStepFlag             1
time_stamping           hardware
network_transport       UDPv4
delay_mechanism         E2E
clockClass              128
clockAccuracy           0x20
offsetScaledLogVariance 0x436A
priority1               128
priority2               128
domainNumber            0
free_running            0
summary_interval        0

[${IFACE}]
EOF
}

write_ptp_service_units() {
  log "Writing custom ptp4l/phc2sys service units"

  rm -rf /etc/systemd/system/ptp4l.service.d
  rm -rf /etc/systemd/system/phc2sys.service.d

  cat >"${PTP4L_SERVICE_PATH}" <<EOF
[Unit]
Description=PTP boundary/grandmaster service for Ouster sync
After=network-online.target chrony.service
Wants=network-online.target chrony.service

[Service]
Type=simple
ExecStart=/usr/sbin/ptp4l -f ${PTP4L_CONF} -i ${IFACE}
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

  cat >"${PHC2SYS_SERVICE_PATH}" <<EOF
[Unit]
Description=Synchronize PTP hardware clock from system realtime clock
After=ptp4l.service chrony.service
Wants=ptp4l.service chrony.service

[Service]
Type=simple
ExecStart=/usr/sbin/phc2sys -s CLOCK_REALTIME -c ${IFACE} -w
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
}

enable_services() {
  log "Reloading systemd and enabling services"
  systemctl daemon-reload
  systemctl enable chrony.service
  systemctl enable ptp4l.service
  systemctl enable phc2sys.service
  systemctl enable ublox-i2c-chrony-bridge.service
}

start_services() {
  if [[ "${NO_START}" == "1" ]]; then
    warn "Skipping immediate service start/restart as requested"
    return
  fi

  log "Starting/restarting services"
  systemctl restart chrony.service
  systemctl restart ublox-i2c-chrony-bridge.service
  systemctl restart ptp4l.service
  systemctl restart phc2sys.service
}

print_next_steps() {
  printf '\n'
  log "Setup complete."
  log "Installed/updated:"
  log "  - PPS overlay on GPIO ${PPS_GPIO} (physical pin 12 if GPIO18)"
  log "  - chrony SOCK + PPS refclocks"
  log "  - u-blox I2C chrony bridge service"
  log "  - ptp4l/phc2sys services for interface ${IFACE}"

  if [[ "${BOOT_CONFIG_CHANGED}" == "1" ]]; then
    warn "Boot config changed. Reboot the Pi before expecting /dev/pps0 to exist."
  fi

  printf '\n'
  log "After reboot, validate the stack with:"
  log "  sudo ./pi_gps_ptp_diagnostic.sh"
  log "  sudo ./pi_gps_ptp_diagnostic.sh --ouster-host <ouster-ip>"
  printf '\n'
  log "Useful service checks:"
  log "  systemctl status ublox-i2c-chrony-bridge.service"
  log "  systemctl status chrony.service"
  log "  systemctl status ptp4l.service"
  log "  systemctl status phc2sys.service"
  log "  chronyc tracking"
  log "  chronyc sources -v"
  log "  ppstest /dev/pps0"
}

main() {
  require_root
  parse_args "$@"
  install_packages
  configure_boot
  install_bridge_script
  write_bridge_service
  configure_chrony
  configure_ptp4l
  write_ptp_service_units
  enable_services
  start_services
  print_next_steps
}

main "$@"

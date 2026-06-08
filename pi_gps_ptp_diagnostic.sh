#!/usr/bin/env bash

set -u

IFACE="eth0"
I2C_BUS="1"
GPS_ADDR="42"
PPS_DEV="/dev/pps0"
PPS_TEST_SECONDS="5"
CHRONY_MAX_CORRECTION="0.01"
CHRONY_MAX_TRIES="1"
CHRONY_INTERVAL_SECONDS="1"
OUSTER_HOST=""
SHOW_RAW_JSON="0"

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

usage() {
  cat <<'EOF'
Usage:
  ./pi_gps_ptp_diagnostic.sh [options]

Options:
  --iface IFACE                 Ethernet interface for PTP checks (default: eth0)
  --i2c-bus BUS                 I2C bus number (default: 1)
  --gps-addr HEX                GPS I2C address without 0x prefix (default: 42)
  --pps-dev PATH                PPS device path (default: /dev/pps0)
  --pps-test-seconds N          Seconds to watch PPS asserts (default: 5)
  --chrony-max-correction SEC   waitsync remaining correction threshold (default: 0.01)
  --chrony-max-tries N          Number of waitsync tries (default: 1)
  --chrony-interval-sec N       Seconds between chrony waitsync polls (default: 1)
  --ouster-host HOST            Optional Ouster hostname/IP for HTTP API checks
  --show-raw-json               Print full Ouster JSON responses
  -h, --help                    Show this help

Examples:
  ./pi_gps_ptp_diagnostic.sh
  sudo ./pi_gps_ptp_diagnostic.sh --ouster-host 169.254.148.183
EOF
}

section() {
  printf '\n== %s ==\n' "$1"
}

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf '[PASS] %s\n' "$1"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  printf '[WARN] %s\n' "$1"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf '[FAIL] %s\n' "$1"
}

note() {
  printf '[INFO] %s\n' "$1"
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

run_and_echo() {
  local label="$1"
  shift

  echo "\$ $*"
  "$@" 2>&1
  local rc=$?
  echo
  return "$rc"
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --iface)
        IFACE="$2"
        shift 2
        ;;
      --i2c-bus)
        I2C_BUS="$2"
        shift 2
        ;;
      --gps-addr)
        GPS_ADDR="${2#0x}"
        GPS_ADDR="${GPS_ADDR,,}"
        shift 2
        ;;
      --pps-dev)
        PPS_DEV="$2"
        shift 2
        ;;
      --pps-test-seconds)
        PPS_TEST_SECONDS="$2"
        shift 2
        ;;
      --chrony-max-correction)
        CHRONY_MAX_CORRECTION="$2"
        shift 2
        ;;
      --chrony-max-tries)
        CHRONY_MAX_TRIES="$2"
        shift 2
        ;;
      --chrony-interval-sec)
        CHRONY_INTERVAL_SECONDS="$2"
        shift 2
        ;;
      --ouster-host)
        OUSTER_HOST="$2"
        shift 2
        ;;
      --show-raw-json)
        SHOW_RAW_JSON="1"
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

check_runtime_context() {
  section "Runtime Context"
  note "Interface: ${IFACE}"
  note "I2C bus: ${I2C_BUS}"
  note "Expected GPS I2C address: 0x${GPS_ADDR}"
  note "PPS device: ${PPS_DEV}"
  if [[ -n "${OUSTER_HOST}" ]]; then
    note "Ouster host: ${OUSTER_HOST}"
  else
    note "Ouster host: not provided, skipping Ouster HTTP checks"
  fi

  if [[ "${EUID}" -ne 0 ]]; then
    warn "Not running as root. Some checks may fail due to permissions. Prefer: sudo ./pi_gps_ptp_diagnostic.sh ..."
  else
    pass "Running as root"
  fi
}

check_i2c() {
  section "I2C GPS Check"

  if [[ ! -e "/dev/i2c-${I2C_BUS}" ]]; then
    fail "/dev/i2c-${I2C_BUS} is missing. Enable I2C and verify the bus number."
    return
  fi
  pass "Found /dev/i2c-${I2C_BUS}"

  if ! command_exists i2cdetect; then
    warn "i2cdetect not found. Install with: sudo apt install i2c-tools"
    return
  fi

  local out
  out="$(i2cdetect -y "${I2C_BUS}" 2>&1 || true)"
  echo "${out}"
  echo

  if echo "${out}" | grep -qiE "(^|[[:space:]])${GPS_ADDR}([[:space:]]|$)"; then
    pass "Detected a device at 0x${GPS_ADDR} on I2C bus ${I2C_BUS}"
  else
    fail "Did not detect 0x${GPS_ADDR} on I2C bus ${I2C_BUS}"
  fi
}

check_pps() {
  section "PPS Check"

  if [[ ! -e "${PPS_DEV}" ]]; then
    fail "${PPS_DEV} does not exist. Check dtoverlay=pps-gpio and wiring."
    return
  fi
  pass "Found ${PPS_DEV}"

  if ! command_exists ppstest; then
    warn "ppstest not found. Install with: sudo apt install pps-tools"
    return
  fi

  local out
  out="$(timeout "${PPS_TEST_SECONDS}" ppstest "${PPS_DEV}" 2>&1 || true)"
  echo "${out}"
  echo

  local assert_count
  assert_count="$(printf '%s\n' "${out}" | grep -c "assert" || true)"
  if [[ "${assert_count}" -ge 2 ]]; then
    pass "Observed ${assert_count} PPS assert events on ${PPS_DEV}"
  else
    fail "Did not observe enough PPS assert events on ${PPS_DEV}. Take the Pi outside and confirm the GPS is outputting PPS."
  fi
}

check_gpsd() {
  section "GPSD Check"

  if command_exists pgrep; then
    local pids
    pids="$(pgrep -af gpsd || true)"
    if [[ -n "${pids}" ]]; then
      echo "${pids}"
      pass "gpsd process is running"
    else
      warn "gpsd process not found"
    fi
  fi

  if command_exists systemctl; then
    local status
    status="$(systemctl is-active gpsd 2>/dev/null || true)"
    if [[ "${status}" == "active" ]]; then
      pass "gpsd service is active"
    elif [[ -n "${status}" ]]; then
      warn "gpsd service state: ${status}"
    fi
  fi

  if command_exists gpspipe; then
    note "Trying a short gpspipe sample..."
    local out
    out="$(timeout 6 gpspipe -r -n 12 2>&1 || true)"
    echo "${out}"
    echo
    if printf '%s\n' "${out}" | grep -qE '^\$G.|^\$P'; then
      pass "gpspipe returned GPS/NMEA data"
    else
      warn "gpspipe did not return NMEA data. This can be normal if gpsd is not configured yet."
    fi
  else
    warn "gpspipe not found. Install gpsd-clients if you want a live GPS feed check."
  fi
}

check_i2c_bridge() {
  section "I2C Chrony Bridge Check"

  if command_exists pgrep; then
    local pids
    pids="$(pgrep -af ublox_i2c_chrony_bridge.py || true)"
    if [[ -n "${pids}" ]]; then
      echo "${pids}"
      pass "u-blox I2C chrony bridge process is running"
    else
      warn "u-blox I2C chrony bridge process not found"
    fi
  fi

  if command_exists systemctl; then
    local status
    status="$(systemctl is-active ublox-i2c-chrony-bridge.service 2>/dev/null || true)"
    if [[ "${status}" == "active" ]]; then
      pass "ublox-i2c-chrony-bridge.service is active"
    elif [[ -n "${status}" ]]; then
      warn "ublox-i2c-chrony-bridge.service state: ${status}"
    fi
  fi

  if [[ -S /run/chrony/zedf9p.sock ]]; then
    pass "Found chrony SOCK refclock socket: /run/chrony/zedf9p.sock"
  else
    warn "chrony SOCK refclock socket /run/chrony/zedf9p.sock not found"
  fi
}

check_chrony() {
  section "Chrony Clock Sync Check"

  if ! command_exists chronyc; then
    warn "chronyc not found. Install/configure chrony before relying on clock sync."
    return
  fi

  local tracking
  tracking="$(chronyc tracking 2>&1 || true)"
  echo "${tracking}"
  echo

  local sources
  sources="$(chronyc sources -v 2>&1 || true)"
  echo "${sources}"
  echo

  if printf '%s\n' "${tracking}" | grep -q "Leap status[[:space:]]*:[[:space:]]*Normal"; then
    pass "chrony reports Leap status: Normal"
  else
    warn "chrony is not yet reporting Leap status: Normal"
  fi

  if printf '%s\n' "${sources}" | grep -qE '^[#\^=]\*'; then
    pass "chrony has a selected synchronization source"
  else
    warn "chrony does not yet show a selected source (*)"
  fi

  local wait_out rc
  wait_out="$(chronyc waitsync "${CHRONY_MAX_TRIES}" "${CHRONY_MAX_CORRECTION}" 0 "${CHRONY_INTERVAL_SECONDS}" 2>&1)"
  rc=$?
  echo "${wait_out}"
  echo

  if [[ "${rc}" -eq 0 ]]; then
    pass "chronyc waitsync succeeded (remaining correction <= ${CHRONY_MAX_CORRECTION}s)"
  else
    warn "chronyc waitsync did not succeed yet. This usually means the Pi clock is not fully disciplined."
  fi
}

check_ptp_stack() {
  section "PTP Stack Check"

  if ! command_exists ip; then
    warn "'ip' command not found"
    return
  fi

  run_and_echo "Interface link state" ip link show "${IFACE}" || true

  if ! command_exists ethtool; then
    warn "ethtool not found. Install with: sudo apt install ethtool"
  else
    local ethtool_out
    ethtool_out="$(ethtool -T "${IFACE}" 2>&1 || true)"
    echo "\$ ethtool -T ${IFACE}"
    echo "${ethtool_out}"
    echo

    if printf '%s\n' "${ethtool_out}" | grep -q "PTP Hardware Clock:[[:space:]]*[0-9]"; then
      pass "${IFACE} exposes a PTP Hardware Clock"
    else
      fail "${IFACE} does not appear to expose a usable PTP Hardware Clock"
    fi
  fi

  if command_exists pgrep; then
    local ptp4l_pids phc2sys_pids

    ptp4l_pids="$(pgrep -af ptp4l || true)"
    if [[ -n "${ptp4l_pids}" ]]; then
      echo "${ptp4l_pids}"
      pass "ptp4l process is running"
    else
      warn "ptp4l process not found"
    fi

    phc2sys_pids="$(pgrep -af phc2sys || true)"
    if [[ -n "${phc2sys_pids}" ]]; then
      echo "${phc2sys_pids}"
      pass "phc2sys process is running"
    else
      warn "phc2sys process not found"
    fi
  fi
}

json_summary_python() {
  local file_path="$1"
  local label="$2"

  python3 - "$file_path" "$label" <<'PY'
import json
import sys

path = sys.argv[1]
label = sys.argv[2]

def find_key(obj, wanted):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == wanted:
                return v
            result = find_key(v, wanted)
            if result is not None:
                return result
    elif isinstance(obj, list):
        for item in obj:
            result = find_key(item, wanted)
            if result is not None:
                return result
    return None

with open(path, "r", encoding="utf-8") as fh:
    data = json.load(fh)

print(f"{label} summary:")
for key in (
    "timestamp_mode",
    "port_state",
    "master_offset",
    "offset_from_master",
    "grandmaster_identity",
    "steps_removed",
):
    value = find_key(data, key)
    if value is not None:
        print(f"  {key}: {value}")
PY
}

check_ouster() {
  section "Ouster HTTP Time/PTP Check"

  if [[ -z "${OUSTER_HOST}" ]]; then
    note "No Ouster host provided. Skipping."
    return
  fi

  if ! command_exists curl; then
    warn "curl not found. Install with: sudo apt install curl"
    return
  fi

  local tmp_time tmp_ptp
  tmp_time="$(mktemp)"
  tmp_ptp="$(mktemp)"

  local rc_time rc_ptp
  curl -fsS "http://${OUSTER_HOST}/api/v1/time" >"${tmp_time}" 2>"${tmp_time}.err"
  rc_time=$?
  curl -fsS "http://${OUSTER_HOST}/api/v1/time/ptp" >"${tmp_ptp}" 2>"${tmp_ptp}.err"
  rc_ptp=$?

  if [[ "${rc_time}" -eq 0 ]]; then
    pass "Fetched /api/v1/time from Ouster"
    if command_exists python3; then
      json_summary_python "${tmp_time}" "/api/v1/time"
    fi
    if [[ "${SHOW_RAW_JSON}" == "1" ]]; then
      cat "${tmp_time}"
      echo
    fi
  else
    warn "Failed to fetch /api/v1/time from Ouster"
    cat "${tmp_time}.err"
    echo
  fi

  if [[ "${rc_ptp}" -eq 0 ]]; then
    pass "Fetched /api/v1/time/ptp from Ouster"
    if command_exists python3; then
      json_summary_python "${tmp_ptp}" "/api/v1/time/ptp"
    fi
    if [[ "${SHOW_RAW_JSON}" == "1" ]]; then
      cat "${tmp_ptp}"
      echo
    fi

    if grep -qi '"port_state"[[:space:]]*:[[:space:]]*"SLAVE"' "${tmp_ptp}"; then
      pass "Ouster PTP port_state looks like SLAVE"
    else
      warn "Ouster PTP port_state is not clearly SLAVE yet"
    fi
  else
    warn "Failed to fetch /api/v1/time/ptp from Ouster"
    cat "${tmp_ptp}.err"
    echo
  fi

  rm -f "${tmp_time}" "${tmp_time}.err" "${tmp_ptp}" "${tmp_ptp}.err"
}

print_next_steps() {
  section "Interpretation"

  if [[ "${FAIL_COUNT}" -eq 0 && "${WARN_COUNT}" -eq 0 ]]; then
    note "Everything in this diagnostic passed."
    note "Next field step: start chrony/PTP services, verify Ouster PTP lock, then test capture."
    return
  fi

  if [[ "${FAIL_COUNT}" -gt 0 ]]; then
    note "Fix FAIL items first. Typical order:"
    note "1. Make sure I2C sees the GPS at 0x${GPS_ADDR}"
    note "2. Make sure ${PPS_DEV} exists and ppstest shows asserts"
    note "3. Make sure chrony reports Leap status: Normal"
    note "4. Make sure ptp4l/phc2sys are running"
    note "5. If Ouster host is provided, verify /api/v1/time/ptp shows a healthy slave state"
  else
    note "No hard failures, but one or more readiness checks are still not healthy."
    note "Most likely you still need more sky time, GPSD/chrony configuration, or PTP service startup."
  fi
}

print_summary() {
  section "Summary"
  printf 'PASS: %d\n' "${PASS_COUNT}"
  printf 'WARN: %d\n' "${WARN_COUNT}"
  printf 'FAIL: %d\n' "${FAIL_COUNT}"
}

main() {
  parse_args "$@"
  check_runtime_context
  check_i2c
  check_pps
  check_gpsd
  check_i2c_bridge
  check_chrony
  check_ptp_stack
  check_ouster
  print_summary
  print_next_steps
}

main "$@"

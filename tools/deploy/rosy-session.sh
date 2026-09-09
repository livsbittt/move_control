#!/usr/bin/env bash
# Manage existing user services on the robot; never install or enable units.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: bash rosy-session.sh boot-minimal
       bash rosy-session.sh start|stop|status map|imu|led

boot-minimal disables and stops only map, LED and IMU services.
Bringup, ADC and control remain unchanged. Verify control ExecStart uses:
  profile:=sensing start_imu:=false calibration_sensing_only:=true
This script does not configure control, install units or enable services.
EOF
}

if [[ $# == 1 && ($1 == --help || $1 == -h) ]]; then
  usage
  exit 0
fi

if [[ $# == 1 && $1 == boot-minimal ]]; then
  result=0
  # Handle each independently so a missing unit cannot conceal another result.
  for capability in map led imu; do
    unit="rosy-session-${capability}.service"
    printf 'Disabling and stopping %s\n' "$unit"
    if timeout 20s systemctl --user disable --now "$unit"; then
      printf 'OK: %s disabled and stopped\n' "$unit"
    else
      printf 'FAILED: %s; inspect state below\n' "$unit" >&2
      result=1
    fi
    timeout 5s systemctl --user show "$unit" --property=LoadState,UnitFileState,ActiveState,SubState || result=1
  done
  printf 'Bringup, ADC and control were not changed. Verify the control sensing profile before reboot.\n'
  exit "$result"
fi

if [[ $# != 2 ]]; then usage >&2; exit 2; fi
case "$1" in start|stop|status) ;; *) usage >&2; exit 2 ;; esac
case "$2" in map|imu|led) ;; *) usage >&2; exit 2 ;; esac
limit=20s
if [[ $1 == status ]]; then limit=5s; fi
exec timeout "$limit" systemctl --user "$1" "rosy-session-${2}.service"

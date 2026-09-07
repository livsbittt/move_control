#!/usr/bin/env bash
# Boot the gz rig, health-check it, retry on bad boots. Keep the healthy
# boot running; print its attempt number and exit 0.
#   bash tools/gz/boot_until_healthy.sh [max_tries]
# (no set -u: ROS setup.bash aborts on it)
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
# Default domain 13; a concurrent rig overrides ROS_DOMAIN_ID (and still
# needs its own GZ_PARTITION — domain IDs do not partition gz transport).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}" ROS_LOCALHOST_ONLY=1 \
       ROS_NETWORK_INTERFACE=lo GZ_IP=127.0.0.1
MAX=${1:-6}

log() { echo "[boot-loop] $*"; }
# Teardown kills ONLY our own runner's process group. Pattern-based
# pkill is partition-blind: it killed the domain-14 rig mid-run once
# already (2026-09-07). A parallel rig must never see a sweep from us.
cleanup() {
  if [ -n "${RUNNER:-}" ] && kill -0 "$RUNNER" 2>/dev/null; then
    kill -TERM -- "-$RUNNER" 2>/dev/null
  fi
  # The server self-detaches from the runner's group: kill it by PID.
  if [ -f "${KILLFILE:-/dev/null}" ]; then
    while read -r P; do
      [ -n "$P" ] && kill -TERM "$P" 2>/dev/null
    done < "$KILLFILE"
  fi
  sleep 2
  if [ -n "${RUNNER:-}" ]; then kill -KILL -- "-$RUNNER" 2>/dev/null; fi
  if [ -f "${KILLFILE:-/dev/null}" ]; then
    while read -r P; do
      [ -n "$P" ] && kill -KILL "$P" 2>/dev/null
    done < "$KILLFILE"
  fi
  # Bridge children orphan when the `ros2 run` wrapper dies (15 leaked
  # once). Env-scoped kill: anything carrying OUR partition env is ours,
  # by exact PID — never a name pattern.
  for pe in /proc/[0-9]*/environ; do
    if tr '\0' '\n' < "$pe" 2>/dev/null \
        | grep -q '^GZ_PARTITION=pinky_rig13$' 2>/dev/null; then
      kill -9 "${pe#/proc/}" 2>/dev/null
    fi
  done
}
# Leftover runners from OUR previous invocations (the pidfile holds ALL
# of them) are the only pre-existing things we may touch — never a
# pattern sweep. Overwriting the pidfile leaked older groups once: two
# slams ran interleaved on one domain and the map corrupted.
PIDFILE=/tmp/gztest/runners.pids
if [ -f "$PIDFILE" ]; then
  while read -r OLD; do
    if [ -n "$OLD" ] && kill -0 "$OLD" 2>/dev/null; then
      log "stopping leftover runner $OLD (our previous boot)"
      RUNNER="$OLD"; cleanup
    fi
  done < "$PIDFILE"
  rm -f "$PIDFILE"
fi
rm -rf /tmp/gztest/monitor /tmp/gztest/gz.log
# gz-sim-main self-detaches into its OWN process group at startup, so a
# PGID kill can never catch it (measured: stacked servers, pgid=self,
# ppid=systemd). Record the real PIDs after every boot and kill them
# by PID in cleanup — world-name-scoped, never a pattern sweep.
KILLFILE=/tmp/gztest/rig.pids
: > "$KILLFILE"
capture() {
  pgrep -f "tools/gz/pinky_maze.sdf" >> "$KILLFILE" 2>/dev/null || true
}
for try in $(seq 1 "$MAX"); do
  log "attempt $try: booting rig"
  setsid bash tools/gz/test_planning.sh > /tmp/gztest/runner.log 2>&1 &
  RUNNER=$!
  echo "$RUNNER" >> "$PIDFILE"
  sleep 105   # boot + slam activation window
  if python3 tools/gz/rig_health.py 3; then
    log "attempt $try HEALTHY — keeping it"
    log "runner pid $RUNNER (kill this + children to stop)"
    wait $RUNNER
    exit 0
  fi
  log "attempt $try unhealthy, cleaning up"
  capture
  kill "$RUNNER" 2>/dev/null
  cleanup
done
log "NO HEALTHY BOOT in $MAX tries"
exit 1

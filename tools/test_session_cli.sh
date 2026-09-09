#!/usr/bin/env bash
# No real systemctl calls: exercise the CLI with a PATH-local fake.
set -euo pipefail
script="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/deploy/rosy-session.sh"
scratch=$(mktemp -d)
trap 'rm -rf -- "$scratch"' EXIT
export SESSION_TEST_LOG="$scratch/calls"
export SESSION_TEST_TIMEOUT_LOG="$scratch/timeouts"
cat > "$scratch/timeout" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$SESSION_TEST_TIMEOUT_LOG"
if [[ ${SESSION_TEST_TIMEOUT_MAP:-0} == 1 && $* == *rosy-session-map.service* ]]; then exit 124; fi
shift
exec "$@"
EOF
cat > "$scratch/systemctl" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$SESSION_TEST_LOG"
if [[ ${SESSION_TEST_FAIL_MAP:-0} == 1 && $* == *'disable --now rosy-session-map.service'* ]]; then exit 1; fi
EOF
chmod +x "$scratch/systemctl" "$scratch/timeout"
export PATH="$scratch:$PATH"
bash -n "$script"
for action in start stop status; do
  for capability in map imu led; do
    bash "$script" "$action" "$capability"
  done
done
[[ $(wc -l < "$SESSION_TEST_LOG") == 9 ]]
grep -q '^20s systemctl --user start rosy-session-map.service$' "$SESSION_TEST_TIMEOUT_LOG"
grep -q '^5s systemctl --user status rosy-session-map.service$' "$SESSION_TEST_TIMEOUT_LOG"
for args in 'start all' 'enable imu' 'stop control' 'boot-minimal map' ''; do
  if bash "$script" $args > /dev/null 2>&1; then exit 1; fi
done
[[ $(wc -l < "$SESSION_TEST_LOG") == 9 ]]
: > "$SESSION_TEST_LOG"
bash "$script" boot-minimal > "$scratch/output"
[[ $(wc -l < "$SESSION_TEST_LOG") == 6 ]]
! grep -Eq 'bringup|adc|control|--user enable' "$SESSION_TEST_LOG"
grep -q -- '--user disable --now rosy-session-imu.service' "$SESSION_TEST_LOG"
: > "$SESSION_TEST_LOG"
if SESSION_TEST_FAIL_MAP=1 bash "$script" boot-minimal > "$scratch/output" 2>&1; then exit 1; fi
grep -q 'FAILED: rosy-session-map.service' "$scratch/output"
grep -q 'OK: rosy-session-imu.service' "$scratch/output"
[[ $(wc -l < "$SESSION_TEST_LOG") == 6 ]]
if SESSION_TEST_TIMEOUT_MAP=1 bash "$script" start map; then exit 1; else [[ $? == 124 ]]; fi
: > "$SESSION_TEST_LOG"
if SESSION_TEST_TIMEOUT_MAP=1 bash "$script" boot-minimal > "$scratch/output" 2>&1; then exit 1; fi
grep -q 'FAILED: rosy-session-map.service' "$scratch/output"
grep -q 'OK: rosy-session-imu.service' "$scratch/output"
[[ $(wc -l < "$SESSION_TEST_LOG") == 4 ]]
printf 'Session CLI syntax and mocked command tests passed.\n'

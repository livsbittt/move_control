#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../.."
export RIG_RENDERED_CAMERA=1 RIG_TRACK_OBSTACLES=1
export RIG_DURATION="${RIG_DURATION:-400}" RIG_WALL_TIMEOUT="${RIG_WALL_TIMEOUT:-1800}"
export RIG_REALTIME_FACTOR="${RIG_REALTIME_FACTOR:-1.0}"
export RIG_REQUIRE_COMPLETE="${RIG_REQUIRE_COMPLETE:-1}"
exec bash tools/gz/run_track260905.sh

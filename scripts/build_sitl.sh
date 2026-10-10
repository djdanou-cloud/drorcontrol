#!/usr/bin/env bash
# Build ArduPilot SITL from source (prebuilt binaries from firmware.ardupilot.org are not
# reachable from every environment). Pinned tag; output binary path printed at the end.
#
# Usage: scripts/build_sitl.sh [WORKDIR] [TAG] [VEHICLE]
#   WORKDIR  where to clone/build (default: .sitl/ in the repo; gitignored)
#   TAG      ArduPilot git tag (default: Copter-4.7.1)
#   VEHICLE  waf target: copter | plane | rover | sub | antennatracker (default: copter)
# Needs: git, g++, python3 (a private venv is created for ArduPilot's build tools).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${1:-$ROOT/.sitl}"
TAG="${2:-Copter-4.7.1}"
VEHICLE="${3:-copter}"
SRC="$WORK/ardupilot-$TAG"

mkdir -p "$WORK"
if [ ! -d "$SRC/.git" ]; then
  git clone --quiet --depth 1 --branch "$TAG" https://github.com/ArduPilot/ardupilot "$SRC"
fi
if [ ! -x "$WORK/venv/bin/python" ]; then
  python3 -m venv "$WORK/venv"
  "$WORK/venv/bin/pip" install --quiet "empy==3.3.4" pexpect future setuptools dronecan
fi
cd "$SRC"
git submodule update --init --depth 1 --recursive \
  modules/waf modules/mavlink modules/DroneCAN/DSDL modules/DroneCAN/dronecan_dsdlc \
  modules/DroneCAN/libcanard modules/DroneCAN/pydronecan modules/littlefs
export PATH="$WORK/venv/bin:$PATH"
python ./waf configure --board sitl --no-submodule-update > "$WORK/configure-$VEHICLE.log" 2>&1 \
  || { tail -40 "$WORK/configure-$VEHICLE.log"; exit 1; }
python ./waf "$VEHICLE" > "$WORK/build-$VEHICLE.log" 2>&1 || { tail -40 "$WORK/build-$VEHICLE.log"; exit 1; }
BIN="$SRC/build/sitl/bin/ardu$VEHICLE"
[ "$VEHICLE" = antennatracker ] && BIN="$SRC/build/sitl/bin/antennatracker"
echo "SITL binary: $BIN"

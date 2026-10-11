#!/usr/bin/env bash
# Build spike S0-3: QMapLibre from a pinned commit (Metal on macOS, OpenGL elsewhere), then the
# harness. Needs Qt >= 6.5 (QT_ROOT_DIR or qt-cmake on PATH), cmake, ninja; ccache if present.
# Usage: ui/spikes/s0-3-map/build.sh   → ui/spikes/s0-3-map/build/spike-s0-3-map(.app)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
DEPS="$HERE/.deps"
# maplibre-native-qt main after "Upgrade Quick to Drawables Renderer (Metal / Vulkan / OpenGL3+)"
# (#216); v3.0.0, the last release, renders through OpenGL only.
MLN_QT_REPO=https://github.com/maplibre/maplibre-native-qt
MLN_QT_SHA=c3485f3a9590081c7be8edbc542341e688f6663d

if [[ -n "${QT_ROOT_DIR:-}" ]]; then
  QT_CMAKE="$QT_ROOT_DIR/bin/qt-cmake"
else
  QT_CMAKE="$(command -v qt-cmake || true)"
fi
[[ -x "$QT_CMAKE" ]] || { echo "qt-cmake not found: set QT_ROOT_DIR to a Qt >= 6.5 install" >&2; exit 1; }

case "$(uname -s)" in
  Darwin) BACKEND=(-DMLN_WITH_METAL=ON) ;;
  *)      BACKEND=(-DMLN_WITH_OPENGL=ON) ;;
esac
LAUNCHER=()
command -v ccache >/dev/null && LAUNCHER=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)

SRC="$DEPS/maplibre-native-qt"
if [[ "$(git -C "$SRC" rev-parse HEAD 2>/dev/null || true)" != "$MLN_QT_SHA" ]]; then
  rm -rf "$SRC"
  git init -q "$SRC"
  git -C "$SRC" remote add origin "$MLN_QT_REPO"
  git -C "$SRC" fetch -q --depth 1 origin "$MLN_QT_SHA"
  git -C "$SRC" checkout -q FETCH_HEAD
  git -C "$SRC" submodule update -q --init --recursive --depth 1
fi

"$QT_CMAKE" -S "$SRC" -B "$DEPS/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$DEPS/install" \
  -DMLN_QT_WITH_LOCATION=OFF -DMLN_QT_WITH_WIDGETS=OFF "${BACKEND[@]}" "${LAUNCHER[@]}"
cmake --build "$DEPS/build"
cmake --install "$DEPS/build" >/dev/null

"$QT_CMAKE" -S "$HERE" -B "$HERE/build" -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$DEPS/install" "${LAUNCHER[@]}"
cmake --build "$HERE/build"
echo "built: $(ls -d "$HERE"/build/spike-s0-3-map* | head -1)"

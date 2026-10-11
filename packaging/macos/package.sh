#!/usr/bin/env bash
# Build the universal (arm64 + x86_64) DrorControl.app, ad-hoc sign it, verify it and package it
# as .zip and .dmg with SHA-256 checksums (ADR-0010: no Apple credentials, no notarization).
#
# Usage (macOS with Xcode command-line tools and the pinned Rust toolchain):
#   packaging/macos/package.sh [OUT_DIR]        # default: dist/
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${1:-$ROOT/dist}"
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
cd "$ROOT"

VERSION="$(cargo metadata --no-deps --format-version 1 \
  | python3 -c 'import json,sys; print(next(p["version"] for p in json.load(sys.stdin)["packages"] if p["name"]=="dc-cli"))')"
BUILD="$(git rev-parse --short=12 HEAD 2>/dev/null || echo local)"
NAME="DrorControl-$VERSION-$BUILD-macos-universal"
export MACOSX_DEPLOYMENT_TARGET=13.0

echo "==> build arm64 + x86_64"
for t in aarch64-apple-darwin x86_64-apple-darwin; do
  rustup target add "$t" >/dev/null
  cargo build --release --locked -p dc-cli --target "$t"
done

echo "==> assemble DrorControl.app"
STAGE="$OUT/stage"; rm -rf "$STAGE"; mkdir -p "$STAGE"
APP="$STAGE/DrorControl.app"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
lipo -create -output "$APP/Contents/MacOS/dc-cli" \
  target/aarch64-apple-darwin/release/dc-cli target/x86_64-apple-darwin/release/dc-cli
sed -e "s/@VERSION@/$VERSION/" -e "s/@BUILD@/$BUILD/" packaging/macos/Info.plist.in > "$APP/Contents/Info.plist"
plutil -lint "$APP/Contents/Info.plist"
printf 'APPL????' > "$APP/Contents/PkgInfo"

echo "==> ad-hoc sign (inside-out: binaries first, then the bundle)"
codesign --force --sign - --timestamp=none "$APP/Contents/MacOS/dc-cli"
codesign --force --sign - --timestamp=none "$APP"

echo "==> verify"
codesign --verify --deep --strict --verbose=2 "$APP"
codesign --display --verbose=2 "$APP" 2>&1 | grep -E "^(Identifier|Signature|Format)" || true
ARCHS="$(lipo -archs "$APP/Contents/MacOS/dc-cli")"
echo "architectures: $ARCHS"
[[ " $ARCHS " == *" arm64 "* && " $ARCHS " == *" x86_64 "* ]] || { echo "not universal: $ARCHS"; exit 1; }
"$APP/Contents/MacOS/dc-cli" --version
# Expected for an ad-hoc build: Gatekeeper does not accept it (users click "Open Anyway").
spctl --assess --type execute --verbose "$APP" 2>&1 || echo "(expected: not notarized, see ADR-0010)"

echo "==> package"
cp packaging/macos/FIRST-LAUNCH.txt "$STAGE/"
rm -f "$OUT/$NAME.zip" "$OUT/$NAME.dmg"
ditto -c -k --sequesterRsrc --keepParent "$APP" "$OUT/$NAME.zip"
hdiutil create -quiet -volname DrorControl -srcfolder "$STAGE" -ov -format UDZO "$OUT/$NAME.dmg"
(cd "$OUT" && shasum -a 256 "$NAME.zip" "$NAME.dmg" > SHA256SUMS && cat SHA256SUMS)
rm -rf "$STAGE"
echo "done: $OUT"

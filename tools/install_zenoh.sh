#!/usr/bin/env bash
# zenoh-c (prebuilt, standalone) and the zenoh-cpp headers into $1 (default /opt/zenoh), the version the C++ API is
# built against: for a local C++ build and for the CI. zenoh-python in pyproject.toml is the same 1.x line.
set -euo pipefail
ZENOH_VERSION="${ZENOH_VERSION:-1.10.1}"
PREFIX="${1:-/opt/zenoh}"
ARCH="$(uname -m)-unknown-linux-gnu"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$PREFIX"
curl -sSfL -o "$WORK/zenoh-c.zip" \
  "https://github.com/eclipse-zenoh/zenoh-c/releases/download/${ZENOH_VERSION}/zenoh-c-${ZENOH_VERSION}-${ARCH}-standalone.zip"
python3 -m zipfile -e "$WORK/zenoh-c.zip" "$PREFIX"
curl -sSfL -o "$WORK/zenoh-cpp.tar.gz" "https://github.com/eclipse-zenoh/zenoh-cpp/archive/refs/tags/${ZENOH_VERSION}.tar.gz"
tar xzf "$WORK/zenoh-cpp.tar.gz" -C "$WORK"
cmake -S "$WORK/zenoh-cpp-${ZENOH_VERSION}" -B "$WORK/zenoh-cpp-build" -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$PREFIX" -DZENOHCXX_ZENOHPICO=OFF > /dev/null
cmake --install "$WORK/zenoh-cpp-build" > /dev/null
echo "zenoh-c and zenoh-cpp ${ZENOH_VERSION} in ${PREFIX}"

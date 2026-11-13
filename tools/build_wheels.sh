#!/usr/bin/env bash
# The files for PyPI: the wheels of blick for CPython 3.10 to 3.13, repaired by auditwheel (manylinux_2_28_x86_64),
# and the sdist. The CI's job wheels and the workflow publish run it; it runs in quay.io/pypa/manylinux_2_28_x86_64:
#
#   tools/build_wheels.sh <out dir> <work dir>
#
# <out dir> gets the wheels and the sdist only. <work dir> must not exist or be empty: the build directories go there,
# so the checkout stays unchanged. Network: the build requirements (scikit-build-core, nanobind, nvidia-cuda-runtime for
# cuda.h) and, for the import check, the dependencies of blick.
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "usage: $0 <out dir> <work dir>" >&2
  exit 2
fi
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$1"
WORK="$2"
PLAT=manylinux_2_28_x86_64
PYTHONS="cp310-cp310 cp311-cp311 cp312-cp312 cp313-cp313"

if [ "${AUDITWHEEL_PLAT:-}" != "$PLAT" ]; then
  echo "run this in quay.io/pypa/${PLAT} (AUDITWHEEL_PLAT is '${AUDITWHEEL_PLAT:-}')" >&2
  exit 1
fi
if [ -e "$WORK" ] && [ -n "$(ls -A "$WORK")" ]; then
  echo "work dir $WORK is not empty" >&2
  exit 1
fi
mkdir -p "$WORK" "$OUT"
WORK="$(cd "$WORK" && pwd)"
OUT="$(cd "$OUT" && pwd)"

for tag in $PYTHONS; do
  "/opt/python/${tag}/bin/python" -m pip wheel --quiet --no-deps --no-cache-dir "$ROOT" -w "$WORK/wheelhouse" \
    -C build-dir="$WORK/build/{wheel_tag}"
done
for wheel in "$WORK"/wheelhouse/*.whl; do
  auditwheel repair --plat "$PLAT" "$wheel" -w "$OUT" > /dev/null
done
/opt/python/cp312-cp312/bin/python -m venv "$WORK/tools"
"$WORK/tools/bin/pip" install --quiet build
"$WORK/tools/bin/python" -m build --sdist --outdir "$OUT" "$ROOT" > /dev/null
# the sdist carries sources only: an output directory inside the checkout must not get into it
if tar tzf "$OUT"/blick-*.tar.gz | grep -q "\.whl$"; then
  echo "the sdist contains wheels (an output directory inside the checkout that pyproject.toml does not exclude)" >&2
  exit 1
fi


for tag in $PYTHONS; do
  env="$WORK/check-${tag}"
  "/opt/python/${tag}/bin/python" -m venv "$env"
  "$env/bin/pip" install --quiet --find-links "$OUT" --only-binary blick blick
  (cd / && "$env/bin/python" -c "import blick, blick._core as c; print('${tag}', blick.__version__, c.host_id()[:8])")
done
ls -l "$OUT"

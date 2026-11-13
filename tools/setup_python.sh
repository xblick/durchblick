#!/usr/bin/env bash
# The Python package and its commands (xblick record, play, rerun, ...) in .venv, as uv.lock fixes them, with the
# extras record and rerun: for recording and for PCs that run no node of their own. Builds the C++ core into it
# (scikit-build-core). For workspace tools that set up the package; UV: the uv to use.
set -euo pipefail
cd "$(dirname "$0")/.."
UV=${UV:-uv}
"$UV" sync --locked --no-dev --extra record --extra rerun
.venv/bin/xblick record --help > /dev/null
echo "xblick: $(.venv/bin/python -c 'import xblick; print(xblick.__version__)') in $(pwd)/.venv"

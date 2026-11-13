#!/usr/bin/env bash
# The Python package and its commands (blick record, play, rerun, ...) in .venv, as uv.lock fixes them, with the
# extras record and rerun: for recording and for PCs that run no node of their own. Builds the C++ core into it
# (scikit-build-core). For workspace tools that set up the package; UV: the uv to use.
set -euo pipefail
cd "$(dirname "$0")/.."
UV=${UV:-uv}
"$UV" sync --locked --no-dev --extra record --extra rerun
.venv/bin/durchblick record --help > /dev/null
echo "durchblick: $(.venv/bin/python -c 'import blick; print(blick.__version__)') in $(pwd)/.venv"

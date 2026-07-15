#!/usr/bin/env bash
# Single source of truth for environment setup -- called by both the root
# Makefile (`make setup`) and .github/workflows/ci.yml, so the bootstrap
# steps never drift between local and CI use.
#
# Creates ./venv (if missing), installs requirements.txt into it, and builds
# the native APE planner shared library (native/ape_ops/build/libape_ops.so)
# that nav/ape_native.py loads via ctypes -- both steps are required before
# run.py can import successfully.
set -euo pipefail
cd "$(dirname "$0")/.."

if [ ! -d venv ]; then
    python3 -m venv venv
fi

./venv/bin/pip install --upgrade pip
./venv/bin/pip install -r requirements.txt

make -C native/ape_ops native

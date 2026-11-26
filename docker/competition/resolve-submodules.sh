#!/usr/bin/env bash
set -euo pipefail
# Only optional GLIM viewer sources need git submodules; commits come from the parent snapshot.
if [ "${WITH_VIEWER:-OFF}" != ON ] || [ "$(cat BACKEND)" != glim ]; then exit 0; fi
python3 tools/fetch_viewer_submodules.py

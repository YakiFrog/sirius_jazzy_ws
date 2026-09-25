#!/usr/bin/env bash
# Roboteq 同定ツールのラッパー。ROS の source は不要。
set -eo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${DIR}/roboteq_id.py" "$@"

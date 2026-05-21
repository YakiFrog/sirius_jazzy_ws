#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
$DIR/venv/bin/python3 $DIR/solix_monitor.py "$@"

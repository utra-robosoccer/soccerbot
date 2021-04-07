#!/bin/bash

source $(dirname "$0")/setenvs.sh
python3 $(dirname "$0")/start_single.py "$@"
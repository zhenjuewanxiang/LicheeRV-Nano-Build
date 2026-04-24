#!/bin/sh
# Sets LD_LIBRARY_PATH to find SDK shared libraries
export LD_LIBRARY_PATH=/mnt/system/usr/lib:/mnt/system/usr/lib/3rd:$LD_LIBRARY_PATH
exec "$(dirname "$0")/stream_demo" "$@"

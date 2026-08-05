#!/bin/sh
set -eu

app_dir=/root/akars-debug
cd "$app_dir"

# disk.img already provides the matching CVI runtime libraries in /lib.
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    export LD_LIBRARY_PATH="/lib:$LD_LIBRARY_PATH"
else
    export LD_LIBRARY_PATH=/lib
fi

# 无参数时直接启动默认捡球流程；带参数时仍可使用 armshow、armtest、
# stopset、centerset 等全部调试子命令。
if [ "$#" -eq 0 ]; then
    set -- models/yolov8n_tennis_v4.cvimodel
fi

exec "$app_dir/akars" "$@"

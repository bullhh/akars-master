#!/bin/sh

app_dir=/root/akars-debug
enable_file="$app_dir/autostart.enabled"

if [ ! -f "$enable_file" ]; then
    echo "[akars-autostart] disabled: $enable_file is absent"
    exit 0
fi

if pidof akars >/dev/null 2>&1; then
    echo "[akars-autostart] akars is already running"
    exit 0
fi

cd "$app_dir" || {
    echo "[akars-autostart] missing directory: $app_dir"
    exit 1
}

# Give the USB event pump a short window to finish enumerating the camera.
sleep 1

attempt=1
while [ "$attempt" -le 3 ]; do
    echo "[akars-autostart] starting attempt $attempt/3"
    "$app_dir/run.sh"
    status=$?
    if [ "$status" -eq 0 ]; then
        echo "[akars-autostart] stopped normally; not restarting"
        exit 0
    fi

    echo "[akars-autostart] exited status=$status"
    attempt=$((attempt + 1))
    if [ "$attempt" -le 3 ]; then
        sleep 1
    fi
done

echo "[akars-autostart] failed after 3 attempts"
exit 1

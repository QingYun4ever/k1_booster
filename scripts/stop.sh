#!/bin/bash
# Allow brain to cancel skills before DDS exits; never kill unrelated Python jobs.
for node in brain_node game_controller_node vision_node sound_play_node; do
    echo "[STOP ${node}]"
    killall -TERM "$node" 2>/dev/null || true
done
for node in brain_node game_controller_node vision_node sound_play_node; do
    attempts=0
    while killall -0 "$node" 2>/dev/null; do
        if [ "$attempts" -ge 50 ]; then
            echo "${node} did not exit; refusing to start a duplicate process" >&2
            exit 1
        fi
        sleep 0.1
        attempts=$((attempts + 1))
    done
done

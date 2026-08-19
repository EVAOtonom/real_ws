#!/usr/bin/env bash
set -e
NODE="${1:-/global_costmap/global_costmap}"
ros2 param set "$NODE" rear_keepout_layer.enabled true

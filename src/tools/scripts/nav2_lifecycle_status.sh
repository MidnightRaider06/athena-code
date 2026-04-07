#!/usr/bin/env bash
# Query the lifecycle state of all Nav2 lifecycle nodes.
#
# Usage:
#   ./nav2_lifecycle_status.sh

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORKSPACE_DIR="$( realpath "$SCRIPT_DIR/../../../../.." )"
source "$WORKSPACE_DIR/install/setup.bash"

# Matches lifecycle_nodes in nav2_nodes.launch.py, plus the lifecycle manager itself
NAV2_NODES=(
    controller_server
    smoother_server
    planner_server
    behavior_server
    bt_navigator
    waypoint_follower
    velocity_smoother
    lifecycle_manager_navigation
)

echo "=========================================="
echo " Nav2 Lifecycle Node Status"
echo "=========================================="

any_found=0
for node in "${NAV2_NODES[@]}"; do
    state=$(ros2 lifecycle get "/$node" 2>/dev/null)
    if [[ -z "$state" ]]; then
        printf "  %-40s  NOT RUNNING\n" "/$node"
        continue
    fi
    any_found=1
    printf "  %-40s  %s\n" "/$node" "$state"
done

if [[ $any_found -eq 0 ]]; then
    echo ""
    echo "  No lifecycle nodes responded. Is Nav2 running?"
fi

echo "=========================================="

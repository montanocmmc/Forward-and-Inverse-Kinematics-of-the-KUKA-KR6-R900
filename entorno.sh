#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

KINEMATICS_WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$KINEMATICS_WS_DIR/install/setup.bash" ]; then
    source "$KINEMATICS_WS_DIR/install/setup.bash"
fi

unset KINEMATICS_WS_DIR

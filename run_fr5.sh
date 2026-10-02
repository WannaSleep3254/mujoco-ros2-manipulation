#!/usr/bin/env bash
# Compatibility entry point for the shared simulator's default FR5 profile.
export MANIPULATION_ROS_DOMAIN_ID="${FR5_ROS_DOMAIN_ID:-${MANIPULATION_ROS_DOMAIN_ID:-89}}"
exec bash "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/run_sim.sh" robot:=fr5 "$@"

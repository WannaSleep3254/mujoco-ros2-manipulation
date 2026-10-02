# Compatibility environment; FR5_ROS_DOMAIN_ID still overrides the default domain.
export MANIPULATION_ROS_DOMAIN_ID="${FR5_ROS_DOMAIN_ID:-${MANIPULATION_ROS_DOMAIN_ID:-89}}"
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/manipulation_env.sh"

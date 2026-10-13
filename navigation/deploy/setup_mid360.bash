# Source this file, do not execute it in a separate shell.
sentry_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$sentry_repo_root/navigation/build_mid360/install/setup.bash"
source "$sentry_repo_root/navigation/build_mid360/lio-install/setup.bash"
export LD_LIBRARY_PATH="$sentry_repo_root/navigation/build_mid360/sdk-build/sdk_core:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR=/tmp/sentry-mid360-ros-logs
unset sentry_repo_root

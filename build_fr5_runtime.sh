#!/usr/bin/env bash
# Rebuild the project-local shutdown fixes without changing /opt/ros or system packages.
set -euo pipefail
task_fr5_build_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$task_fr5_build_root"

if [[ ! -d external/mujoco_ros2_control ]]; then
  git clone --depth 1 --branch 0.1.2 \
    https://github.com/ros-controls/mujoco_ros2_control.git external/mujoco_ros2_control
fi
task_fr5_native_commit="$(git -C external/mujoco_ros2_control rev-parse HEAD)"
if [[ "$task_fr5_native_commit" != 178b1c39e5010185116bc89b4e94cbb25b825f61 ]]; then
  echo 'Unexpected MuJoCo ROS source revision; the shutdown patch targets version 0.1.2.' >&2
  exit 1
fi

if git -C external/mujoco_ros2_control apply --check \
    "$task_fr5_build_root/patches/mujoco_ros2_control-0.1.2-shutdown.patch" 2>/dev/null; then
  git -C external/mujoco_ros2_control apply \
    "$task_fr5_build_root/patches/mujoco_ros2_control-0.1.2-shutdown.patch"
elif ! git -C external/mujoco_ros2_control apply --reverse --check \
    "$task_fr5_build_root/patches/mujoco_ros2_control-0.1.2-shutdown.patch" 2>/dev/null; then
  echo 'The source has other changes; cannot safely apply the shutdown patch.' >&2
  exit 1
fi

# Reuse the official deb files already downloaded for this PC.
for task_fr5_cached_deb in runtime/debs/*.deb; do
  dpkg-deb -x "$task_fr5_cached_deb" runtime/root
done
if [[ ! -f runtime/root/usr/include/GLFW/glfw3.h ||
      ! -f runtime/root/opt/ros/humble/share/ros2_control_cmake/cmake/ros2_control_cmakeConfig.cmake ]]; then
  echo 'Missing build dependencies: download libglfw3-dev and ros-humble-ros2-control-cmake into runtime/debs.' >&2
  exit 1
fi

mkdir -p runtime/native
gcc -shared -fPIC -O2 -Wall -Wextra -Werror \
  -o runtime/native/fr5_plugin_lifetime.so native/fr5_plugin_lifetime.c -ldl -pthread

# ROS environment hooks can refer to unset variables.
set +u
source ./fr5_env.sh
set -u
export CMAKE_PREFIX_PATH="$task_fr5_build_root/runtime/root/usr:$task_fr5_build_root/runtime/root/opt/ros/humble:${CMAKE_PREFIX_PATH:-}"
colcon --log-base ros_ws/log build \
  --base-paths external/mujoco_ros2_control/mujoco_ros2_control \
  --build-base ros_ws/build --install-base ros_ws/install \
  --packages-select mujoco_ros2_control \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo

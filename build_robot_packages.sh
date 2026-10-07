#!/usr/bin/env bash
# 제조사 모델 및 시뮬레이션용 MoveIt 패키지만 프로젝트 안에 빌드합니다.
set -euo pipefail
task_robot_build_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$task_robot_build_root"

task_fetch_model_source() {
  local task_source_directory="$1" task_source_url="$2" task_source_revision="$3"
  if [[ ! -d "$task_source_directory" ]]; then
    mkdir -p "$task_source_directory"
    git -C "$task_source_directory" init --quiet
    git -C "$task_source_directory" remote add origin "$task_source_url"
    git -C "$task_source_directory" fetch --depth 1 origin "$task_source_revision"
    git -C "$task_source_directory" checkout --detach FETCH_HEAD
  fi
  if [[ "$(git -C "$task_source_directory" rev-parse HEAD)" != "$task_source_revision" ]]; then
    echo "Source revision mismatch: $task_source_directory; existing checkout left unchanged." >&2
    return 1
  fi
}

task_fetch_model_source external/frcobot_ros2 \
  https://github.com/FAIR-INNOVATION/frcobot_ros2.git \
  fcf0c7f0d60d949d8a9a4238f929a44d07f60379
task_fetch_model_source external/ur_description \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git \
  65fa221f6d9e1904b30b6a05ae39cb24a0c40fac

set +u
source ./manipulation_env.sh
set -u
colcon --log-base ros_ws/log build \
  --base-paths external/frcobot_ros2/fairino_description \
               external/frcobot_ros2/fairino5_v6_moveit2_config \
               external/frcobot_ros2/fairino10_v6_moveit2_config \
               external/ur_description packages/ur5e_moveit_config \
  --build-base ros_ws/build --install-base ros_ws/install \
  --packages-select fairino_description fairino5_v6_moveit2_config \
                    fairino10_v6_moveit2_config ur_description ur5e_moveit_config \
  --cmake-args -DBUILD_TESTING=OFF

for task_robot_id in fr5 fr10 ur5e; do
  .venv/bin/python prepare_robot.py --robot "$task_robot_id"
done

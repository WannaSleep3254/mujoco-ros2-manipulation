# 단일 관절 ROS 토픽 예제

루트의 기존 명령은 유지됩니다. 구현은 `src/mujoco_ros2_manipulation/examples/single_joint/`에 있습니다.

```bash
source ./ros2_env.sh
.venv/bin/python mujoco_ros2_bridge.py
# 별도 터미널
source ./ros2_env.sh
.venv/bin/python verify_mujoco_ros2.py
```

이 예제는 직접 Python 토픽 브리지이며, 로봇팔의 ros2_control 경로와 구별됩니다.

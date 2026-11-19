# FR5 ACT 接口仿真

独立 ROS 2 Humble 工作空间，使用 RViz + ros2_control GenericSystem 模拟硬件。
包含 FR5 模型、MoveIt、原交接包的 IKFast + KDL 插件和 125 Hz 轨迹控制器。
这是运动接口仿真，不是 Gazebo 物理仿真；已加入双指占位夹爪和 DM-Tac W2M 真实尺寸视觉模型；不包含相机图像、物体接触动力学或 ACT 模型。详见 [GRIPPER.md](GRIPPER.md)。
没有复制真机驱动或 SDK。脚本固定 ROS_DOMAIN_ID=85、ROS_LOCALHOST_ONLY=1。
使用系统 Python 3.10 运行 ROS，ACT 的 Conda Python 3.12 单独保留。

## 安装与构建

在 Ubuntu 22.04 终端运行（安装依赖时输入本机 sudo 密码）：

```bash
cd /data/Projects/fr5_sim
bash scripts/install_dependencies.sh
bash scripts/build.sh
bash scripts/start_sim.sh
```

RViz 的 MotionPlanning 面板选择 `fairino5_v6_group`，可以选命名姿态 `pos1` / `pos2` 或拖动目标，先 Plan，再 Execute。
当前初始关节位置沿用交接模型的零位；若起点有碰撞，需检查模型/初始姿态，不能关闭碰撞检查来绕过。

第二个终端运行接口测试：

```bash
bash /data/Projects/fr5_sim/scripts/test_sim.sh
```

测试先检查 controller_manager 实际加载且已激活的硬件确为 GenericSystem，然后读取关节状态、执行小幅 j1 往返轨迹并测试取消。
轨迹测试直接调用 FollowJointTrajectory，用于验证控制接口，不验证 MoveIt 规划或碰撞。
启动时不会自动执行轨迹。Ctrl+C 关闭启动终端可结束仿真。
无桌面时可用 `bash scripts/start_sim.sh use_rviz:=false`。

## 验证状态

`python3 scripts/check_files.py` 可在未安装 ROS 时检查文件哈希、脚本语法、模型网格和 mock 插件。
编译、RViz 显示、IK 插件加载、规划和轨迹集成测试，必须在安装 ROS 后实际执行；静态检查不能代替这些结果。

原始模型与 IK 源码来自 fr5_act_handoff_20261114_232532_15208.zip，source_manifest.json 记录所复制文件的 SHA-256。
原真机工作空间未修改。仿真不调用任何原 start_moveit.sh 或硬件 launch。

## 2026-11-15 本机验证记录

- ROS 2 Humble 下四个包编译通过。
- RViz、MoveIt 和 125 Hz 模拟控制器已启动；IKFast + KDL 插件成功加载。
- 启动日志记录到一次成功的 MoveIt 规划请求。
- test_sim.sh 通过：模拟硬件身份、关节反馈、连续两段往返轨迹和取消操作。
- 未验证 Gazebo 物理接触、相机、夹爪、ACT 推理或真机。
- 无 3D 传感器/物体识别服务的提示不影响此次关节接口测试。

# FR5 + DM-Tac W2M 仿真

选用用户确认的 DM-Tac W2M。两侧各一只暂定，使用用户提供 STEP 转换的真实尺寸 STL，单位从毫米转换为米。
夹爪底座、夹指和安装连接为占位设计，尺寸不代表厂家规格。传感器安装朝向尚未按实物确认。
这仍是 RViz + GenericSystem 运动接口仿真，不包含物体接触动力学、触觉图像、力或摩擦模拟。

## 启动和开合

```bash
bash /data/Projects/fr5_sim/scripts/start_sim.sh
# 另一个终端：80 mm 全开 / 20 mm 开口
bash /data/Projects/fr5_sim/scripts/gripper.sh 0.08
bash /data/Projects/fr5_sim/scripts/gripper.sh 0.02
# 开合、恢复和取消测试
bash /data/Projects/fr5_sim/scripts/gripper.sh --test
```

宽度单位米，范围 0–0.08；表示两侧传感器保守碰撞包络之间的距离，尚不是标定后的触觉接触面距离。
两个夹指使用独立滑动关节，脚本同步发送 width/2。直接向控制器发送任意不同关节值并不受机械同步约束。
动作接口 `/gripper_controller/follow_joint_trajectory`。关节 `gripper_left_joint`、`gripper_right_joint`。
RViz 可选择 `gripper` 组的 open/closed 目标。碰撞检测保持启用；零间隙目标可能因接触边界被拒绝。
FR5 原 IK 链仍以 wrist3_link 为末端；新增 gripper_tcp 为虚拟工具参考点，未把原 IKFast 改成另一条链。

## 参数和几何

`src/fr5_act_sim/config/gripper.yaml` 定义安装平移/旋转、夹指尺寸和行程。
`mount_xyz=[0,0,0.12]` 相对于 wrist3_link；网格端面约在该坐标系 z=0.099 m，额外距离是临时安装预留，非真实法兰设计。
W2M 全模型包络（含建模线缆）约 36.00 × 74.47 × 47.91 mm。
视觉使用真实网格；碰撞使用含线缆的保守包围盒，可能拒绝实际能通过的狭窄路径。
传感器朝向由 sensor_visual_rpy 分侧配置。按用户截图，下侧（初始姿态的右侧）已沿传感器长轴翻转 180 度；上侧保持原朝向。
CAD 保留原始模型，并另存居中网格；转换哈希、尺寸、平移记录位于 `meshes/tactile/*json`。

资料检查：SDK_Publish_V1.2.13.1 为触觉传感器 SDK，示例含图像、形变、深度、剪切和六维力，不据此推断夹爪电机接口。
UI 压缩包为可视化软件及运行环境。未安装/运行厂家程序或连接设备。
W2L 模型也已转换供参考。DM-Tax X 的 STEP 文件在 ORIENTED_EDGE 定义中途结束，无法解析，未生成虚假替代网格。

## 2026-11-15 验证

- 四包构建通过，RViz 和两个轨迹控制器启动，IKFast + KDL 加载。
- 当前装配 GetStateValidity=true，无碰撞接触。
- 带 W2M 的机械臂 OMPL 规划成功，返回 25 个轨迹点。
- 夹爪 80 mm → 4 mm → 60 mm 反馈、取消通过；机械臂往返和取消回归通过。
- 启动不发送自动动作。未做真机和触觉物理验证。
- 已观察到 MoveIt 在 Ctrl+C 退出阶段发生析构段错误（夹爪加入前也出现）；运行期间测试通过，退出异常仍待单独排查。

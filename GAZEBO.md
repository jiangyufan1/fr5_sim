# Gazebo Fortress 桌面仿真

已接入 ROS 2 Humble + Gazebo Fortress，FR5 固定在 0.75 m 高桌面上，带双 W2M 网格、可动占位夹爪、测试方块和外置腕部 RGB 相机。相机有可见外壳与镜头，朝向夹指前方桌面区域。

启动：`bash /data/Projects/fr5_sim/scripts/start_gazebo.sh`
查看相机：`bash /data/Projects/fr5_sim/scripts/view_camera.sh`
夹爪开到 60 mm：`bash /data/Projects/fr5_sim/scripts/gazebo_gripper.sh 0.06`

Gazebo 使用 ROS_DOMAIN_ID=86，旧 mock 仿真使用 85，两者命令不能混用。当前机器默认软件渲染以避免显卡驱动兼容问题，速度可能低于实时。
相机话题 `/wrist_camera/image`、`/wrist_camera/camera_info`；光学坐标 `wrist_camera_optical_frame`。分辨率 640×480，设定 15 Hz；实际帧率受仿真性能影响。
参数在 `src/fr5_act_sim/config/gazebo_scene.yaml`：桌面、底座固定点、相机安装位姿和测试方块位置。

2026-11-15 实测通过：Gazebo 插件、仿真时钟、八关节反馈、非空 RGB 图像；夹爪开合/恢复/取消；当前状态无碰撞，含桌面的 OMPL 规划返回 25 个轨迹点。相机画面已读取验证，测试方块置于其可见区域。

夹爪/法兰、相机尺寸与质量惯量是临时设计；W2M 碰撞使用含线缆的保守包围盒。触觉力场、触觉图像和抓取成功率未验证。位置控制不代表真实夹持力控制；此场景不能作为真实抓取性能结论。

ROS Humble 的 Gazebo 插件在硬件列表中返回空 class_type，检查脚本改为核对活动状态与 robot_description 的 Gazebo 插件名。
关闭 Gazebo 后应在启动终端 Ctrl+C，让 ROS 节点一并结束。

## 五物品入盒场景（仅布置阶段）

5 个 40 mm 彩色动态方块（object_1…object_5），质量各 50 g；固定敞口盒 collection_bin 外尺寸 280×260 mm，高 100 mm，壁厚 10 mm。盒子由底板和四壁组成，没有顶盖。
布局在 gazebo_scene.yaml 的 task 下配置：seed=42，可复现随机位置，改 seed 后重启得到新布局。最小中心间距 85 mm。
已验证 Gazebo 实际存在全部物品和盒子，MoveIt 中有对应初始碰撞对象，机械臂初始状态无碰撞。未执行抓取，也尚未逐物品验证抓取姿态可达性。
当前腕部相机视野有限，未保证一次能看见全部物品。物品移动后的 MoveIt 位姿同步、抓取接触与成功判定留待下一步流程实现；当前碰撞对象为初始化快照。

# MotorSport CARLA → ROS 2 bridge

This package publishes the virtual 40-channel CARLA LiDAR as
`sensor_msgs/PointCloud2` on `/carla/lidar/points`. Stable cone tracks are
published as `visualization_msgs/MarkerArray` on `/detected_cones`. Paired
left/right cones are converted into centre points and published as
`nav_msgs/Path` on `/planned_path`. Repeated stable cones are merged into a
bounded local map and published on `/cone_map`. The H3 pure-pursuit controller
publishes `geometry_msgs/Twist` on `/path_following/cmd_vel`.

本包把 CARLA 虚拟 40 线 LiDAR 发布为 `/carla/lidar/points`，并把稳定锥桶轨迹
发布为 `/detected_cones`。左右锥桶配对后的中点路径发布为 `/planned_path`。
重复稳定锥桶合并为有界局部地图并发布到 `/cone_map`。配套 RViz 配置会同时
显示点云、红色锥桶编号、蓝色累计地图和橙色中心路径。H3 纯追踪控制器在
`/path_following/cmd_vel` 发布 `geometry_msgs/Twist` 控制命令。

The H2 planner selects the nearest cross-track cone pair, moves to its
midpoint, and updates the next local heading. It rejects track widths outside
2–8 m and requires at least two pairs. It is verified on straight and curved
local corridors; H4 extends the interface to a pre-mapped closed track with
continuous cone-landmark relocalization and lap-level evaluation.

H2 规划器选择最近的横向锥桶对，移动到其中点，再更新下一段局部方向；拒绝
2–8 米范围外的赛道宽度，并要求至少两对锥桶。H4 已把该接口扩展到预建图
闭环赛道，并加入持续锥桶地标重定位和圈级评估。

H4 loads a prebuilt circular cone-landmark map, injects deterministic
odometry drift, corrects it with live LiDAR landmark matches, and evaluates a
complete lap. Its bicycle model isolates the perception–localization–
planning–control loop; H3 remains the separate CARLA dynamics test.

H4 载入预建圆形锥桶地标地图，注入确定性里程计漂移，使用实时 LiDAR 地标匹配
进行校正，并评估完整一圈。其自行车模型用于单独评估感知—定位—规划—控制
闭环；H3 仍作为独立 CARLA 动力学测试。

H5 generalises the closed track to circle, oval, and asymmetric technical
loops. It adds repeatable cone dropout/noise, incremental multi-lap
relocalization, a bounded dynamic-obstacle safety override, and a locked
prebuilt map that refines matched landmarks without accepting unmatched noisy
ones. Selected live CARLA runs reuse a saved centre route and use corrected
localization to follow it.

H5 把闭环赛道推广到圆形、椭圆形和非对称技术型赛道，加入可重复的锥桶漏检/
噪声、增量多圈重定位、有界动态障碍安全覆盖，以及“只细化已匹配地标、不接收
未匹配噪声”的预建地图锁定。选定实时 CARLA 测试复用保存的中心路线，并使用
校正后的定位进行路径跟随。

Automated tests / 自动测试：

- `ros2_ws/test_path_integration.sh` — straight regression / 直线回归
- `ros2_ws/test_curve_path_integration.sh` — curved H2 integration / H2弯道集成
- `ros2_ws/test_path_following_integration.sh` — CARLA dynamics closed loop / CARLA动力学闭环
- `ros2_ws/test_closed_loop_lap.sh` — H4 full lap and relocalization / H4完整圈与重定位
- `tools/run_h5_robustness.py` — H5 deterministic eight-scenario matrix / H5确定性8场景矩阵
- `ros2_ws/test_h5_carla_integration.sh` — H5 selected live circle/oval validation / H5选定实时圆形/椭圆验证

H5 latest baseline / H5 最新基线：

- Deterministic matrix: 8/8 scenarios, 24/24 laps, worst P95 cross-track error
  0.531 m. / 确定性矩阵：8/8 场景、24/24 圈通过，最差 P95 横向误差 0.531 米。
- Live CARLA circle: 3/3 laps, mean/P95 cross-track error 0.084/0.168 m. /
  实时 CARLA 圆形：3/3 圈，平均/P95 横向误差 0.084/0.168 米。
- Live CARLA oval: 1/1 lap, mean/P95 cross-track error 0.246/0.558 m. /
  实时 CARLA 椭圆：1/1 圈，平均/P95 横向误差 0.246/0.558 米。

The default run is 1000 simulation frames (50 seconds at 20 FPS) and 2 m/s.
默认运行 1000 帧（20 FPS 下约 50 秒），车速为 2 m/s。

## Public real-world data / 公开真实数据

The converted LeGO-LOAM VLP-16 sample is stored under `datasets/processed/`.
Double-click `replay_public_lidar.cmd` to replay it in RViz2. CARLA is not
required.

转换后的 LeGO-LOAM VLP-16 样本保存在 `datasets/processed/`。双击
`replay_public_lidar.cmd` 即可在 RViz2 中回放，不需要启动 CARLA。

## Public LiDAR odometry / 公开点云里程计

KISS-ICP `v1.3.0` is built in the same ROS 2 workspace. Double-click
`run_public_lidar_odometry.cmd` to replay the public bag at 0.5 speed, show the
local map and pose trail, and save `output/public_kiss_icp_trajectory.csv`.

同一 ROS 2 工作区内已经编译 KISS-ICP `v1.3.0`。双击
`run_public_lidar_odometry.cmd` 后，会以 0.5 倍速回放公开数据，显示局部地图和
位姿轨迹，并保存 `output/public_kiss_icp_trajectory.csv`。整个过程不需要 CARLA。

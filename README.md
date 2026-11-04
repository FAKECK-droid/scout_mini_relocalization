# Scout Mini small_gicp 最小重定位

基于固定 PCD 地图、RViz 粗位姿和 FAST-LIO 已去畸变的单帧 body 点云，
持续低频 GICP 配准。仅广播动态 `map → odom`，由 EKF 独占
`odom → base_link`。没有里程计输出、EKF set_pose 调用或运动控制接口。

## 坐标与外参

`T_A_B` 将 B 点变换到 A。map 就是选定 PCD 的原生坐标，加载时不旋转地图。
FAST-LIO 保存的地图属于建图那次运行的 camera_init；请确认地图来源和高度。

- `T_base_lidar` 从 TF buffer 查询静态 `base_link ← lidar`。非零时间戳的
  动态路径不接受，避免把经过 EKF/LIO 两套运动位姿的路径当成安装外参。
- `T_imu_lidar` 从同一 FAST-LIO YAML 的 `mapping.extrinsic_T/R` 读取。
  body 是 IMU，`T_base_body = T_base_lidar × inverse(T_imu_lidar)`。
- 每帧在扫描时间 t 精确查询 `T_odom_base(t)`，不回退到最新 TF。
- `T_map_base = align(map, scan_base, guess).T_target_source`。
- `T_map_odom = T_map_base × inverse(T_odom_base(t))`。

现有 bringup 已强制 FAST-LIO 的 `mapping.extrinsic_est_en=false`。
独立启动时也必须接入使用同一固定 IMU–雷达外参的 LIO；若运行实例
在线估计外参，静态 YAML 无法代表其当前外参，应先关闭在线估计。
本包不新增 body 与 base_link 的 TF 边。

## 构建

通过开发机完整桌面 SSH 环境执行：

```powershell
ssh jetson-orin 'bash -lc "source /opt/ros/humble/setup.bash; source /home/yan/ros2_ws/install/setup.bash; cd /home/yan/ros2_ws; CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --base-paths src/scout_mini_relocalization src/scout_mini_nav --packages-select scout_mini_relocalization scout_mini_nav --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -Dsmall_gicp_DIR=/home/yan/ros2_ws/install/small_gicp/lib/cmake/small_gicp"'
```

small_gicp 使用工程内已安装的 v1.0.1 CMake 库，并非 Humble 的 ROS
ament 包；依赖导出目标 `small_gicp::small_gicp`。无需重新安装系统依赖。

## 启动

已运行现有 FAST-LIO、EKF 和静态安装 TF 时，推荐独立接入：

```bash
source /opt/ros/humble/setup.bash
source /home/yan/ros2_ws/install/setup.bash
ros2 launch scout_mini_relocalization relocalization.launch.py \
  prior_pcd_file:=/home/yan/ros2_ws/src/scout_mini_nav/maps/s402.pcd
```

必须人工选择现场对应的地图，示例路径不代表已验证该地图或现场。
独立 launch 只启动重定位节点。退出后，已有局部里程计继续运行。
`fast_lio_config` 默认使用已安装的 mid360_jetson.yaml，可显式传入实际配置。
`relocalization_config` 指定本包参数 YAML；没有另一套安装外参数值。

完整 bringup 接入：

```bash
ros2 launch scout_mini_nav bringup_launch.py \
  mapping_mode:=false use_relocalization:=true \
  prior_pcd_file:=/home/yan/ros2_ws/src/scout_mini_nav/maps/s402.pcd
```

完整 bringup 沿用原有设备启动默认值，包含底盘进入受控模式；
实车操作遵循项目急停和场地检查约定。本次自动测试仅运行软件节点。

RViz Fixed Frame 设为 `map`，用 2D Pose Estimate 给出 map 中
**base_link** 的粗位姿。默认保留消息的 z；原生地图的车体高度不为零时，
在配置中启用 `use_initial_base_z` 并按地图标定 `initial_base_z`。
默认 1 Hz 配准、20 Hz TF 刷新、2 个计算线程，参数为现场调试起点。

## 接口与失效行为

| 接口 | 方向 | 说明 |
| --- | --- | --- |
| /cloud_registered_body | 订阅 PointCloud2，best effort，深度 1 | 有时间戳的 body 帧、FLOAT32 XYZ；输入话题和 frame 可配 |
| /initialpose | 订阅 PoseWithCovarianceStamped | frame 必须为 map；表达 base_link；默认与最新可用里程计 TF 配对；可切换严格时间戳模式 |
| /tf、/tf_static | 读取 | 静态安装外参、扫描时刻的 EKF TF，以及初始化使用的里程计 TF |
| map → odom | 唯一 TF 输出 | 当前 ROS 时间刷新最近接受的修正，无未来时间偏移 |
| /relocalization/pose | 发布 PoseStamped | 扫描时间的 map 中 base_link 位姿 |
| /relocalization/map_cloud | 发布 PointCloud2 | 降采样先验地图，reliable/transient_local，深度 1 |
| /relocalization/status | 发布 DiagnosticArray | 状态、拒绝原因、内点、米制 RMSE、H 谱、处理耗时、成功时间和修正年龄 |
| /relocalization/aligned_cloud | 可选发布 PointCloud2 | 默认关闭 |

首次成功前不发布单位 map→odom。首次和重新初始化需要两帧质量通过且
map→odom 一致；人工初值只进入试算，不立即替换已接受修正。
新请求递增 generation；旧任务返回后不能提交。最多一个配准任务在途，
回调只留最新扫描，不积压历史扫描。同步 align 不能即时取消，超时结果被丢弃。

质量门限包括收敛、绝对内点数、两种内点比例、独立欧氏最近邻 RMSE、
正常跟踪时的预测位姿修正量、有限数据和时间有效性。人工初始化默认不按
粗初值的修正距离/角度拒收，仍要求质量通过且连续两帧修正一致。
H 的谱只作诊断，不宣称已证明
全局正确或所有退化情形可检测。重复结构仍可能误匹配。

连续失败或结果过旧进入 DEGRADED，短期保持旧修正；超过
`tf_hold_timeout` 停止刷新全局 TF，新的有效匹配可以恢复。
重新初始化未成功时持续重试，旧修正的刷新资格仍受成功年龄限制。
ROS 时间回退、观测到显著 odom 跳变时作废旧修正；未确认的人工初值保留，
重新等待里程计 TF；没有待处理初值时等待新初值。
跳变检测无法识别所有 odom 重置；重启 EKF/LIO 后应重启重定位并重新初始化。
停止刷新不清除其他消费者的 TF 历史，也不执行自动停车。

## 验证与边界

`relocalization_geometry` 验证非单位 EKF、安装外参、初值推进、
map→odom 确认、source→target 配准方向和质量门限。
`relocalization_integration` 在本机隔离 ROS 域 230 中仅运行重定位节点、
合成扫描和模拟 EKF TF，覆盖初值、移动、错误 frame、空扫描、缺失 TF、
重新初始化、修正超时、时间回退、odom 跳变和退出后的局部 TF。
`scout_mini_nav` 的原有几何、真实 EKF 软件融合测试应继续通过；
新增 launch 契约测试验证开关、地图路径、外参数值一致和静态占位移除。

测试生成的日志和合成地图保留在 build 内的测试目录。真实 MID360 录包、
已有地图的现场定位精度、重复走廊、CPU 竞争及 EKF 频率对照仍需要现场验收。
合成数据通过不能代替这些测量。本版本无全局搜索、自动全局恢复、
地图更新、短窗累积或导航控制。

## 2026-10-08 Jetson 验证记录

两个包在 ROS Humble / Jetson Orin Nano 上 Release 编译成功。
以下功能检查均已通过：

- relocalization_geometry：已知 SE(3) 恢复与坐标代数，独立欧氏 RMSE 约 0.059 m。
- relocalization_integration：10 个隔离 ROS 域 230 用例，覆盖初始化、跟踪和失效。
- relocalization_ekf_isolation：在隔离 ROS 域 229 中向真实 robot_localization EKF
  重放相同轮速/LIO 输入和 /clock；开启重定位、重新给初值、退出后对照。
  286 个共同采样点的局部位置最大差 0 m，
  旋转最大差 2.980e-8 rad；接受了明显非零的全局修正，
  odom→base_link 与 /odometry/filtered 一致，退出后 EKF 继续运行。
- scout_mini_nav 原有 lidar_odom_transformer_geometry（7 个用例）、
  wheel_lio_fusion（2 个用例）及新增 relocalization_launch_contract（4 个用例）通过。
- 两种已安装 launch 的 --show-args 解析通过，没有启动设备节点。
- ekf.yaml、lidar_odom_transformer.yaml 与部署前 SHA256 一致。

可复现功能测试：

```bash
source /opt/ros/humble/setup.bash
source /home/yan/ros2_ws/install/setup.bash
cd /home/yan/ros2_ws
ctest --test-dir build/scout_mini_relocalization --output-on-failure
ctest --test-dir build/scout_mini_nav \\
  -R 'lidar_odom_transformer_geometry|wheel_lio_fusion|relocalization_launch_contract' \\
  --output-on-failure
```

数值对照产物：`/home/yan/ros2_ws/build/scout_mini_relocalization/ekf-comparison-wxldcbgx/comparison.json`。

完整 nav 格式检查仍有原有失败：flake8 33 项、pep257 23 项、uncrustify 2 个文件。
已在 build/scout_mini_nav/baseline-lint-l8r16dr1 对原始提交 56a39ba 与本次版本
作独立对照，错误数量相同，本次新增代码没有增加这些错误。
cppcheck（安装版本 2.7 被工具跳过）、lint_cmake 和 xmllint 返回通过。
没有为修复旧格式问题改动里程计源码。

这些结果证明本次合成回放的接口与数值隔离；真实 MID360/PCD 的精度和
CPU 竞争、50 Hz EKF 实流性能仍未验收。没有运行真实底盘或发布运动命令。


## 2026-11-04 RViz 初始位姿无限等待与随时重设

默认 initialpose_use_latest_tf=true：RViz 的 map 初始位姿与最新可用
odom → base_link 配对，避免因点击时刻 TF 尚未到达而在 0.2 秒后丢弃初值。
完全没有里程计 TF 时无限期保留初值，TF 可用后自动开始配准。
配准确认也没有总超时，持续用最新点云重试，直到成功或被新初值覆盖。
已移除 initialization_timeout 参数和两处超时丢弃逻辑。

等待 TF、配准进行中、定位成功后，都可以再次发布 /initialpose。
新请求立即替换旧请求，清理旧的待执行任务；已经运行的旧任务结果按 generation
丢弃，不能覆盖新初值。里程计跳变或 ROS 时间回退时也保留尚未确认的人工请求。

这只改变人工粗初值的时间配对；扫描配准继续查询每帧点云采集时刻的 TF。
initialpose_use_latest_tf=false 可恢复按初始位姿消息时间戳查询。
tf_wait_timeout 继续用于扫描 TF 等待。

终端会打印初值已接收、无限期等待里程计、开始配准、初始化成功或配准拒绝原因。
单帧点云和单次计算的时效、质量检查继续生效，失败不会丢弃人工初值。
初始化成功前仍不发布假的单位 map → odom。
RViz 的 Fixed Frame 应为 map，2D Pose Estimate 发布到 /initialpose。
map → odom 建立后，依赖该变换的 RViz 消息才能解除等待。

本轮只构建安装，按用户要求不运行测试或启动硬件。当前停止程序时不会有
/initialpose 订阅者；请在重定位节点启动并显示 Map loaded 后再点击初值。

## 2026-11-04 FAST-LIO 积压导致 RViz 丢消息

现场只读测量：Livox 原始点云约延迟 0.12 秒、IMU 约 1 毫秒，
FAST-LIO 的点云和里程计约延迟 1.37 秒；EKF 的 odom → base_link 约延迟
5 毫秒。重定位已收到 /initialpose，但点云超过 max_scan_age=0.5 秒，
因此在配准前被拒绝，map → odom 无法建立或在旧修正过期后停止刷新。

FAST-LIO 的雷达订阅改为 SensorDataQoS / keep_last(1)，内部待处理扫描由
max_buffered_scans（默认 2）限制；丢弃旧扫描时同步处理时间队列和
lidar_pushed 状态，保留 IMU 数据用于连续积分。未改写采集时间戳。
重定位拒收日志和 /relocalization/status 新增实际 cloud_age_s 诊断。
初始位姿仍无限期保留，初始化和重设继续使用同一个 /initialpose 话题。

本轮只编译安装，未运行测试；重启原导航 bringup 后使用新二进制。


## 2026-11-04 人工初值 correction_jump 拒收修复

人工 /initialpose 用于首次初始化及随时重设，默认
initialpose_check_correction=false，不再用 init_max_translation=1 米、
init_max_rotation=15 度限制人工粗初值与配准结果的差异。
设为 true 可恢复这两项附加限制。

正常跟踪继续使用 track_max_translation=0.5 米、
track_max_rotation=10 度。所有模式保留收敛、有效变换、内点、RMSE 与时效检查；
人工初始化还需连续 confirmation_frames=2 帧的 map → odom 修正一致。
新的人工请求仍替换旧请求，旧任务结果按 generation 丢弃，等待无总超时。

correction_jump 日志现在打印 initialization/tracking 模式、实际修正距离和
角度及各自门限；初始化成功日志打印相对粗初值的修正量。
/relocalization/status 新增 registration_mode、initialpose_check_correction、
correction_translation_m 和 correction_rotation_rad。

本轮只构建安装重定位可执行程序，不运行测试、不重启硬件。

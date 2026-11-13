# Jetson 部署验证记录

日期：2026-11-13。全部开发板操作由开发机通过 `ssh jetson-orin` 执行。

## 版本与安装位置

- 工作空间：`/home/yan/ros2_ws`；ROS 2 Humble。
- Jetson Linux R36.4.4；CUDA 编译器 12.6.68；GPU compute capability 8.7。
- 重定位功能源码提交：`166a5c2fa990c1fa31f1f34019012ecbe2ad9e1d`。
- CUDA 安装与验证工具提交：`c59d1535fe1fce7a6be87cb0aab2043cbf188ecd`。
- GLIM/glim_ros：1.2.2-0jammy，ARM64 CUDA 12.6。
- gtsam_points：1.2.2；GTSAM 软件包 4.3.0（库 SONAME 4.3a1）；Iridescence 1.0.2。
- 独立安装：`~/ros2_ws/tools/glim_cuda/sysroot`，约 66 MB（包含下载缓存与验证记录）。
- 激活：`source ~/ros2_ws/tools/glim_cuda/setup.bash`。
- 官方 PPA 的 Release.gpg 验证通过；签名者 Kenji Koide，
  key fingerprint `A0DAE9CF75CFEF37A509FCD3C2C44E47B79E8F3A`。
- Packages.gz 和全部所选 deb 的 SHA256 验证通过。
- libglfw3 3.3.6-1 从宿主 Ubuntu APT 索引下载并校验，解包到相同私有 sysroot。
  未通过 sudo/dpkg 安装到系统，未修改系统软件源、ldconfig 或 shell 启动文件。

## 重定位包验证

在 Jetson 原工作空间重新构建成功。三组 CTest 全部通过：

- relocalization_geometry；
- relocalization_integration：12 项测试通过；
- relocalization_ekf_isolation：通过，115 个共享样本的最大位置差为 0，
  最大旋转差约 2.98e-8 rad。

测试记录位于 `~/ros2_ws/build/scout_mini_relocalization/Testing/Temporary/LastTest.log`。
现有 launch 和 config 与功能改动前保持一致。

## GLIM CUDA 验证

运行 README 中的 smoke.py，仅使用 DDS domain 228、localhost 和合成点云/IMU：

- CUDA device count：1；compute capability：8.7；
- 实际进程加载 `libodometry_estimation_gpu.so` 和 `libgtsam_points_cuda.so`；
- CUDA 因子库包含原生 `sm_87` cubin；
- 产生 110 条有限、有效的 Odometry 输出；
- 正常退出，未遗留 GLIM 或底盘进程。

原始证据：
`~/ros2_ws/tools/glim_cuda/qa/cuda-smoke-oqxvhbpi/result.json` 和同目录 `node.log`。
该结果只证明 CUDA 部署及计算路径可运行，不是 Femto Bolt 实机精度或性能基准。

## 保留与边界

没有启动真实底盘或发布运动命令，也没有启动相机或修改相机配置。
现有导航仓库的 `params/nav2_params.yaml` 未提交改动原样保留。
真实 Femto Bolt 点云/IMU 的时间、外参及导航 TF 接入尚未配置，需在后续接入任务中验证。
本次临时 SSH 代理只绑定回环地址，未修改路由、DNS、防火墙或 SSH 配置。

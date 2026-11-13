# Jetson 部署验证记录

日期：2026-11-13。全部开发板操作由开发机通过 `ssh jetson-orin` 执行。

## 版本与目录

- 工作空间：`/home/yan/ros2_ws`；ROS 2 Humble。
- Jetson Linux R36.4.4；CUDA 编译器 12.6.68；GPU compute capability 8.7。
- 重定位功能源码提交：`166a5c2fa990c1fa31f1f34019012ecbe2ad9e1d`。
- 标准 ROS 2 部署工具提交：`362f0dd157c5d68f5d59159507edc82aa69515dd`。
- 两端 `src/glim`：官方 v1.2.2，提交 `faa264a1bce1bda406f73457e35511f56cdc2eaa`。
- 两端 `src/glim_ros2`：官方 v1.2.2，提交 `fc72f4603ed2d3dffcdf94cda5617963afe57e82`。
- 两个上游仓库均无源码改动，固定 tag 的 detached HEAD 为预期状态。
- Jetson 标准安装位置：`~/ros2_ws/install/glim` 和 `~/ros2_ws/install/glim_ros`。
- 激活：`source ~/ros2_ws/install/setup.bash`。
- 构建与运行日志分别保留在工作空间的 `build`、`log` 中。

## 系统依赖与构建

已使用授权的 sudo 正常安装官方依赖，不保存密码：

- libgtsam-points-cuda12.6-dev 1.2.2；
- libgtsam-notbb-dev 4.3.0（库 SONAME 4.3a1）；
- libiridescence-dev 1.0.2；
- libglfw3/libglfw3-dev 3.3.6-1、libglm-dev、libmetis-dev；
- libomp-dev 为原有依赖。

官方 PPA 使用单独的 `/usr/share/keyrings/koide3-ppa.gpg`，
软件源为 `/etc/apt/sources.list.d/koide3-glim.list`。
签名者 Kenji Koide，指纹 `A0DAE9CF75CFEF37A509FCD3C2C44E47B79E8F3A`。
依赖位于标准系统目录，并已运行 ldconfig；没有安装 ROS GLIM 二进制包。

按 README 中的命令，以 2 个编译线程、顺序执行器构建 `glim` 和 `glim_ros`：

- Release、BUILD_WITH_CUDA=ON、BUILD_WITH_VIEWER=ON；
- 使用 `/usr/local/cuda-12.6` 和 `/usr/local/lib/cmake/gtsam_points`；
- 两个功能包均构建成功，总耗时约 9 分 18 秒；
- `ros2 pkg prefix` 分别返回上述工作空间 install 路径。

上游 GTSAM/ament 有非致命弃用提示，未出现编译或链接错误。

## 重定位包验证

在 Jetson 原工作空间重新构建成功。三组 CTest 全部通过：

- relocalization_geometry；
- relocalization_integration：12 项测试通过；
- relocalization_ekf_isolation：通过，115 个共享样本的最大位置差为 0，
  最大旋转差约 2.98e-8 rad。

测试记录位于 `~/ros2_ws/build/scout_mini_relocalization/Testing/Temporary/LastTest.log`。
现有 launch 和 config 与功能改动前保持一致。

## GLIM CUDA 验证与旧目录清理

工作空间源码构建后验证通过，再删除本次创建的旧 `~/ros2_ws/tools/glim_cuda`。
删除后再次运行 README 中的 smoke.py，结果仍通过：

- CUDA device count：1；compute capability：8.7；
- 进程实际加载 `install/glim/lib/libodometry_estimation_gpu.so` 和 `libgtsam_points_cuda.so`；
- 进程未加载旧 `tools/glim_cuda` 内的库；
- 仅使用 DDS domain 228、localhost、合成点云和 IMU；
- 产生 110 条有限、有效的 Odometry 输出；
- 静止合成数据的最终位置约为 [-0.000225, -0.000151, -0.000173] m；
- 正常退出，未遗留本次验证进程。

清理后的验证证据：
`~/ros2_ws/log/glim_cuda/cuda-smoke-dh81dwxx/result.json` 和同目录 `node.log`。
前一次源码构建验证证据为 `~/ros2_ws/log/glim_cuda/cuda-smoke-v8mo1egx`。
这些结果只证明 CUDA 部署及计算路径可运行，不是 Femto Bolt 实机精度或性能基准。

## 保留与边界

没有启动真实底盘或发布运动命令，也没有启动相机或修改相机配置。
现有导航仓库的 `params/nav2_params.yaml` 未提交改动原样保留。
真实 Femto Bolt 点云/IMU 的时间、外参及导航 TF 接入尚未配置。
临时 SSH 代理只绑定回环地址，未修改路由、DNS、防火墙或 SSH 配置。

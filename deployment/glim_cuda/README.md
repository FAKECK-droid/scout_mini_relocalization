# GLIM CUDA：标准 ROS 2 功能包部署

两端源代码放在工作空间的 src 中，Jetson 使用 colcon 构建。固定官方 v1.2.2：

| 源码目录 | ROS 2 包名 | 上游与提交 |
| --- | --- | --- |
| src/glim | glim | https://github.com/koide3/glim ，faa264a1bce1bda406f73457e35511f56cdc2eaa |
| src/glim_ros2 | glim_ros | https://github.com/koide3/glim_ros2 ，fc72f4603ed2d3dffcdf94cda5617963afe57e82 |

构建输出使用工作空间原有的 build/glim、build/glim_ros、install/glim、
install/glim_ros 和 log。不向 src 放置编译产物，也不修改现有导航 launch/config。

## 系统依赖

Jetson 为 Ubuntu 22.04、Humble、CUDA 12.6。官方 PPA：
https://koide3.github.io/ppa/ubuntu2204/ 。

通过已授权的 sudo 正常安装 libgtsam-points-cuda12.6-dev=1.2.2、
libiridescence-dev=1.0.2，以及 libglfw3-dev、libglm-dev、libmetis-dev、libomp-dev。
GTSAM 由上述依赖安装。软件源使用单独的 signed-by 密钥；无需自定义运行时 sysroot。
sudo 密码不写入任何脚本、文件或日志。

## Jetson 构建与使用

全部开发板操作从开发机通过 ssh jetson-orin 执行。进入已核实的工作空间后：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --base-paths src/glim src/glim_ros2 --packages-select glim glim_ros \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_CUDA=ON -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TESTING=OFF \
  -DCUDAToolkit_ROOT=/usr/local/cuda-12.6
source install/setup.bash
ros2 pkg prefix glim
ros2 pkg prefix glim_ros
ros2 pkg executables glim_ros
```

## GPU 验证

```powershell
ssh jetson-orin 'source ~/ros2_ws/install/setup.bash && python3 ~/ros2_ws/src/scout_mini_relocalization/deployment/glim_cuda/smoke.py --workspace ~/ros2_ws'
```

验证脚本只接受所选工作空间 install 下的 GLIM，检查实际进程中的 GPU 模块和
CUDA 因子库。使用 DDS domain 228、localhost、合成点云及 IMU；关闭 GUI 和全局建图，
不启动真实底盘或相机。配置与日志位于 log/glim_cuda。

这只验证软件 CUDA 计算路径，不代表真实 Femto Bolt 精度或实时性能。
真实点云/IMU 的时间、外参与话题接入仍需单独配置。

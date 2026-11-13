# Jetson GLIM CUDA 12.6 部署

目标为本项目已确认的 ARM64 Jetson、Ubuntu 22.04、ROS 2 Humble 和 CUDA 12.6。
固定 GLIM/glim_ros/gtsam_points 1.2.2、GTSAM 4.3.0、Iridescence 1.0.2。
来源为作者官方 PPA：https://koide3.github.io/ppa/ubuntu2204/ 。
脚本验证 Release.gpg、Packages.gz 与每个 deb 的 SHA256，并解包到所选工作空间
的 tools/glim_cuda/sysroot；不运行包维护脚本、不修改 /etc、/usr 或 /opt，不需要 sudo。
现有系统运行库仍由宿主提供，安装后必须检查 ldd 和 GPU 模块实际加载结果。

## 从桌面 SSH 执行

先将本仓库提交推送，并在 Jetson git pull --ff-only。目标路径须在执行前核实。

```powershell
ssh jetson-orin 'python3 ~/ros2_ws/src/scout_mini_relocalization/deployment/glim_cuda/install.py --workspace ~/ros2_ws'
```

网络需要桌面代理时，可使用仅绑定回环地址的临时 SSH 反向转发，
再给脚本传入 --proxy http://127.0.0.1:17890；不需要启用 TUN 或修改路由。

```bash
source ~/ros2_ws/tools/glim_cuda/setup.bash
ros2 pkg prefix glim_ros
ros2 pkg executables glim_ros
```

此环境只对 source 它的当前 shell 生效；不修改 shell 启动文件或现有导航 launch。
默认包配置不保证适合 Femto Bolt，不应直接并入底盘 bringup。实机使用前需准备
点云/IMU 的时间与外参、话题和输出 TF 归属，并单独验证所选版本的性能。

## CUDA 运行验证

```powershell
ssh jetson-orin 'source ~/ros2_ws/tools/glim_cuda/setup.bash && python3 ~/ros2_ws/src/scout_mini_relocalization/deployment/glim_cuda/smoke.py'
```

脚本固定使用 DDS domain 228 和 localhost，发布静止三维场景与重力 IMU，
关闭全局建图和 GUI，保留 ROS 输出插件。验证 CUDA context、GPU 里程计模块、
CUDA 因子库及有效 Odometry 输出。所有配置与日志保留在 tools/glim_cuda/qa 下。
该测试仅证明 GPU 运行路径可用，不代表真实 Femto Bolt 的精度或实时性能。

#!/usr/bin/env python3
"""Exercise the deployed GPU odometry using synthetic points/IMU in DDS domain 228."""
import ctypes
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "228"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def load_json(path):
    # Preserve JSON strings while removing upstream JSON-with-comments syntax.
    pattern = r'("(?:\\.|[^"\\])*")|//[^\n]*|/\*[\s\S]*?\*/'
    content = re.sub(pattern, lambda match: match.group(1) or "", path.read_text())
    return json.loads(content)


def main():
    root = Path(os.environ["GLIM_CUDA_ROOT"]).resolve(strict=True)
    qa = root / "qa"
    qa.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix="cuda-smoke-", dir=qa))
    os.environ["ROS_LOG_DIR"] = str(output / "ros-log")
    prefix = root / "sysroot/opt/ros/humble"
    libraries = (prefix / "lib/libodometry_estimation_gpu.so",
                 root / "sysroot/usr/local/lib/libgtsam_points_cuda.so")
    for library in libraries:
        result = subprocess.check_output(["ldd", str(library)], text=True)
        if "not found" in result:
            raise RuntimeError(result)
    cuda = ctypes.CDLL("libcudart.so.12")
    count = ctypes.c_int()
    assert cuda.cudaGetDeviceCount(ctypes.byref(count)) == 0 and count.value > 0
    assert cuda.cudaFree(None) == 0
    major, minor = ctypes.c_int(), ctypes.c_int()
    assert cuda.cudaDeviceGetAttribute(ctypes.byref(major), 75, 0) == 0
    assert cuda.cudaDeviceGetAttribute(ctypes.byref(minor), 76, 0) == 0
    config = output / "config"
    config.mkdir()
    documents = {}
    for path in (prefix / "share/glim/config").glob("*.json"):
        documents[path.name] = load_json(path)
    documents["config_ros.json"]["glim_ros"].update(
        enable_local_mapping=False, enable_global_mapping=False,
        extension_modules=["librviz_viewer.so"], acc_scale=1.0,
        imu_topic="/glim_cuda_smoke/input/imu", points_topic="/glim_cuda_smoke/input/points",
        image_topic="/glim_cuda_smoke/input/image",
        imu_frame_id="glim_test_imu", lidar_frame_id="glim_test_depth",
        base_frame_id="glim_test_imu", odom_frame_id="glim_test_odom",
        map_frame_id="glim_test_map", publish_imu2lidar=False)
    documents["config_sensors.json"]["sensors"].update(
        global_shutter_lidar=True, T_lidar_imu=[0, 0, 0, 0, 0, 0, 1])
    documents["config_odometry_gpu.json"]["odometry_estimation"].update(
        initialization_mode="NAIVE", max_num_keyframes=5, save_imu_rate_trajectory=False)
    documents["config_preprocess.json"]["preprocess"].update(
        random_downsample_target=2000, downsample_resolution=0.10)
    documents["config_logging.json"]["logging"].update(log_dir=str(output / "glim-log"))
    for name, document in documents.items():
        (config / name).write_text(json.dumps(document, indent=2) + "\n")
    rclpy.init()
    node = rclpy.create_node("glim_cuda_smoke_input")
    imu_pub = node.create_publisher(Imu, "/glim_cuda_smoke/input/imu", qos_profile_sensor_data)
    points_pub = node.create_publisher(PointCloud2, "/glim_cuda_smoke/input/points", qos_profile_sensor_data)
    poses = []
    subscription = node.create_subscription(Odometry, "/glim_cuda_smoke/odom", poses.append, 100)
    points = []
    for x in np.linspace(-3, 3, 35):
        for y in np.linspace(-2, 2, 29):
            points.extend([[x, y, -1.0+0.03*math.sin(x)],
                           [x, 3.0+0.05*math.sin(y), y+2.5],
                           [-3.5+0.03*math.cos(x), x, y+2.5]])
    process = None
    log = open(output / "node.log", "w")
    try:
        process = subprocess.Popen([
            str(prefix / "lib/glim_ros/glim_rosnode"), "--ros-args",
            "-r", "__node:=glim_cuda_smoke", "-p", "config_path:=" + str(config),
            "-p", "dump_path:=" + str(output / "dump")], stdout=log, stderr=log)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if process.poll() is not None:
                raise RuntimeError("GLIM exited during startup; inspect " + str(output))
            if imu_pub.get_subscription_count() and points_pub.get_subscription_count():
                break
        else:
            raise RuntimeError("GLIM discovery failed")
        end = time.monotonic() + 12
        next_imu = next_points = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if process.poll() is not None:
                raise RuntimeError("GLIM exited during GPU odometry")
            if now >= next_imu:
                message = Imu()
                message.header.frame_id = "glim_test_imu"
                message.header.stamp = node.get_clock().now().to_msg()
                message.orientation_covariance[0] = -1.0
                message.linear_acceleration.z = 9.80665
                imu_pub.publish(message)
                next_imu += 0.005
            if now >= next_points:
                nanoseconds = time.time_ns() - 20_000_000
                header = Header(frame_id="glim_test_depth",
                                stamp=Time(sec=nanoseconds//10**9, nanosec=nanoseconds%10**9))
                points_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
                next_points += 0.1
            rclpy.spin_once(node, timeout_sec=0.001)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        maps = Path(f"/proc/{process.pid}/maps").read_text()
        assert "libodometry_estimation_gpu.so" in maps
        assert "libgtsam_points_cuda.so" in maps
        assert len(poses) >= 30, f"Only {len(poses)} odometry messages"
        positions = np.array([[p.pose.pose.position.x, p.pose.pose.position.y,
                               p.pose.pose.position.z] for p in poses])
        assert np.isfinite(positions).all()
        assert np.linalg.norm(positions[-1]) < 0.5, positions[-1]
        result = dict(cuda_devices=count.value, compute_capability=f"{major.value}.{minor.value}",
                      gpu_odometry_loaded=True, cuda_factors_loaded=True,
                      odometry_messages=len(poses), final_position_m=positions[-1].tolist(),
                      synthetic=True, artifacts=str(output))
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=20)
        log.close()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        print("Artifacts:", output, flush=True)


if __name__ == "__main__":
    main()

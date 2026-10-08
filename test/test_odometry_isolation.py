"""Replay identical measurements into the real EKF with localization off/on."""
import argparse
import importlib.util
import json
import math
import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

parser = argparse.ArgumentParser()
parser.add_argument("--binary", required=True)
args, remaining = parser.parse_known_args()
project = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "synthetic_helpers", project / "test/test_relocalization_integration.py")
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
os.environ["ROS_DOMAIN_ID"] = "229"
os.environ["ROS_LOCALHOST_ONLY"] = "1"


def spin(node, duration):
    end = time.monotonic()+duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.002)


def ground_truth(seconds):
    matrix = helpers.RelocalizationTest.ob(seconds)
    matrix[2, 3] = 0.0
    return matrix


def run_replay(enabled, directory, world_points, pcd):
    rclpy.init()
    node = rclpy.create_node("ekf_relocalization_comparison_input")
    output, local_tf, global_tf = {}, {}, []
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    wheel_pub = node.create_publisher(Odometry, "/odom", 20)
    lio_pub = node.create_publisher(Odometry, "/lio/odometry", 20)
    static_pub = node.create_publisher(
        TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    cloud_pub = node.create_publisher(PointCloud2, "/cloud_registered_body", qos_profile_sensor_data)
    seed_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def record(message):
        key = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        pose = message.pose.pose
        output[key] = np.r_[[
            pose.position.x, pose.position.y, pose.position.z], [
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]]

    def record_tf(message):
        for tf in message.transforms:
            if tf.child_frame_id == "base_link":
                key = tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
                local_tf[key] = np.r_[[
                    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z], [
                    tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w]]
            if tf.header.frame_id == "map":
                global_tf.append(tf)

    subs = [
        node.create_subscription(Odometry, "/odometry/filtered", record, 100),
        node.create_subscription(TFMessage, "/tf", record_tf, 100)]
    il = helpers.transform([-0.011, -0.02329, 0.04412], [0.09, -0.08, 0.13])
    bl = helpers.transform([0.07, -0.09, 0.145], [0.11, -0.12, -1.1])
    bi = bl @ np.linalg.inv(il)
    mo = helpers.transform([4.0, -3.0, 0.5], [0.04, -0.06, 0.35])
    ekf_config = yaml.safe_load(
        (project.parent / "scout_mini_nav/params/ekf.yaml").read_text())
    ekf_config["ekf_filter_node"]["ros__parameters"]["use_sim_time"] = True
    ekf_file = directory / ("ekf-on.yaml" if enabled else "ekf-off.yaml")
    ekf_file.write_text(yaml.safe_dump(ekf_config))
    log = open(directory / ("nodes-on.log" if enabled else "nodes-off.log"), "w")
    processes = []
    reloc = None
    try:
        ekf = Path(get_package_prefix("robot_localization")) / "lib/robot_localization/ekf_node"
        processes.append(subprocess.Popen(
            [str(ekf), "--ros-args", "-r", "__node:=ekf_filter_node",
             "--params-file", str(ekf_file)], stdout=log, stderr=log))
        if enabled:
            params = dict(
                use_sim_time=True, prior_pcd_file=str(pcd),
                t_imu_lidar=il[:3, 3].tolist(), R_imu_lidar=il[:3, :3].flatten().tolist(),
                map_leaf_size=0.15, scan_leaf_size=0.15, registration_rate=4.0)
            config = directory / "relocalization.yaml"
            config.write_text(yaml.safe_dump({"small_gicp_relocalizer": {"ros__parameters": params}}))
            reloc = subprocess.Popen(
                [args.binary, "--ros-args", "--params-file", str(config)], stdout=log, stderr=log)
            processes.append(reloc)
        static_pub.publish(TFMessage(transforms=[
            helpers.RelocalizationTest.tf("base_link", "lidar", bl, 0.0)]))
        deadline = time.monotonic()+6
        while time.monotonic() < deadline:
            spin(node, 0.05)
            if any(p.poll() is not None for p in processes):
                raise AssertionError("Software node exited")
            if wheel_pub.get_subscription_count() and lio_pub.get_subscription_count():
                if not enabled or cloud_pub.get_subscription_count():
                    break
        else:
            raise AssertionError("DDS discovery timeout")
        clock_pub.publish(Clock(clock=helpers.stamp(1000.0)))
        spin(node, 0.15)
        for step in range(1, 311):
            seconds = 1000.0+step*0.02
            known = ground_truth(seconds)
            q = helpers.quaternion(known)
            wheel = Odometry()
            wheel.header.frame_id, wheel.child_frame_id = "odom", "base_link"
            wheel.header.stamp = helpers.stamp(seconds)
            wheel.pose.pose.position.x = 2000.0  # Ignored wheel pose, as in production.
            wheel.pose.pose.orientation.w = 1.0
            yaw = -0.25+0.015*(seconds-1000.0)
            wheel.twist.twist.linear.x = 0.06*math.cos(yaw)+0.02*math.sin(yaw)
            wheel.twist.twist.angular.z = 0.015
            wheel.twist.covariance[0] = wheel.twist.covariance[35] = 0.01
            wheel_pub.publish(wheel)
            if step % 5 == 0 or step == 1:
                lio = Odometry()
                lio.header, lio.child_frame_id = wheel.header, "base_link"
                p = known[:3, 3]
                lio.pose.pose.position.x, lio.pose.pose.position.y, lio.pose.pose.position.z = map(float, p)
                lio.pose.pose.orientation.x, lio.pose.pose.orientation.y, lio.pose.pose.orientation.z, lio.pose.pose.orientation.w = map(float, q)
                for index in range(6):
                    lio.pose.covariance[7*index] = 0.0025
                lio_pub.publish(lio)
            # Queue sensor measurements before advancing the deterministic clock.
            spin(node, 0.015)
            clock_pub.publish(Clock(clock=helpers.stamp(seconds)))
            if enabled and step < 250:
                scan_time = seconds-0.02
                if step % 5 == 0:
                    source = (np.linalg.inv(bi) @ np.linalg.inv(mo @ ground_truth(scan_time))
                              @ world_points.T).T[:, :3]
                    cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                        Header(stamp=helpers.stamp(scan_time), frame_id="body"), source.tolist()))
                if step in (40, 180):
                    seed = PoseWithCovarianceStamped()
                    seed.header.frame_id, seed.header.stamp = "map", helpers.stamp(scan_time)
                    pose = mo @ ground_truth(scan_time)
                    p, q = pose[:3, 3], helpers.quaternion(pose)
                    seed.pose.pose.position.x, seed.pose.pose.position.y, seed.pose.pose.position.z = map(float, p)
                    seed.pose.pose.orientation.x, seed.pose.pose.orientation.y, seed.pose.pose.orientation.z, seed.pose.pose.orientation.w = map(float, q)
                    seed_pub.publish(seed)
            if enabled and step == 250:
                reloc.terminate()
                reloc.wait(timeout=8)
            spin(node, 0.025)
        spin(node, 0.2)
        tf_nodes = {p.node_name for p in node.get_publishers_info_by_topic("/tf")}
        if "ekf_filter_node" not in tf_nodes:
            raise AssertionError("EKF TF publisher disappeared")
        return output, local_tf, global_tf
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=8)
        log.close()
        for sub in subs:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


class OdometryIsolationTest(unittest.TestCase):
    def test_identical_ekf_replay_with_localization_and_reinit_and_exit(self):
        directory = Path(tempfile.mkdtemp(prefix="ekf-comparison-", dir=Path.cwd()))
        points = []
        for x in np.linspace(-3, 3, 35):
            for y in np.linspace(-2.4, 2.4, 29):
                points.extend([
                    [x, y, 0.12*math.sin(x)+0.08*math.cos(2*y)],
                    [x, 3+0.1*math.sin(y), y+2.5],
                    [-3.4+0.07*math.cos(x), x, y+2.5]])
        pcd = directory / "map.pcd"
        with pcd.open("w") as stream:
            stream.write(
                f"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n")
            np.savetxt(stream, np.array(points), fmt="%.8f")
        world = np.c_[np.array(points), np.ones(len(points))]
        off, off_tf, no_global = run_replay(False, directory, world, pcd)
        on, on_tf, global_tf = run_replay(True, directory, world, pcd)
        self.assertFalse(no_global)
        self.assertGreater(len(global_tf), 10)
        self.assertTrue(all(t.child_frame_id == "odom" for t in global_tf))
        self.assertGreater(abs(global_tf[-1].transform.translation.x), 2.0)
        keys = sorted(set(off) & set(on) & set(off_tf) & set(on_tf))
        keys = [k for k in keys if k >= 1000.5*1e9]
        self.assertGreater(len(keys), 100)
        self.assertTrue(any(k >= 1005.8*1e9 for k in keys), "EKF must continue after localization exit")
        position_difference, yaw_difference = [], []
        for key in keys:
            np.testing.assert_allclose(off[key], off_tf[key], atol=1e-10)
            np.testing.assert_allclose(on[key], on_tf[key], atol=1e-10)
            position_difference.append(np.linalg.norm(off[key][:3]-on[key][:3]))
            q1, q2 = off[key][3:], on[key][3:]
            yaw_difference.append(2*math.acos(min(1.0, abs(float(np.dot(q1, q2))))))
        result = dict(
            shared_samples=len(keys), max_position_delta_m=max(position_difference),
            max_rotation_delta_rad=max(yaw_difference), global_tf_messages=len(global_tf),
            artifacts=str(directory))
        (directory / "comparison.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
        self.assertLessEqual(result["max_position_delta_m"], 0.001)
        self.assertLessEqual(result["max_rotation_delta_rad"], math.radians(0.1))


if __name__ == "__main__":
    unittest.main(argv=[__file__]+remaining)

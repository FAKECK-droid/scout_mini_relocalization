"""Synthetic scans and TF in an isolated DDS domain; no hardware or cmd_vel."""
import argparse
import math
import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "230"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time as RosTime
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

parser = argparse.ArgumentParser()
parser.add_argument("--binary", required=True)
args, remaining = parser.parse_known_args()


def rotation(rpy):
    x, y, z = rpy
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    return np.array([
        [cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
        [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
        [-sy, cy*sx, cy*cx]])


def transform(position, rpy):
    result = np.eye(4)
    result[:3, :3] = rotation(rpy)
    result[:3, 3] = position
    return result


def quaternion(matrix):
    r = matrix[:3, :3]
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ]) / 3
    return np.linalg.eigh(k)[1][:, -1]


def matrix_from_pose(pose):
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return result


def stamp(seconds):
    nanoseconds = round(seconds*1e9)
    return Time(sec=nanoseconds//10**9, nanosec=nanoseconds % 10**9)


class RelocalizationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("relocalization_test_input")
        cls.directory = Path(tempfile.mkdtemp(prefix="relocalization-test-", dir=Path.cwd()))
        cls.log = open(cls.directory / "node.log", "w+")
        cls.process = None
        cls.addClassCleanup(cls.cleanup)
        cls.time = 1000.0
        cls.step = 0
        cls.outputs = []
        cls.transforms = []
        cls.tf_buffer = None
        cls.statuses = []
        cls.maps = []
        cls.local_inputs = {}
        cls.clock_pub = cls.node.create_publisher(Clock, "/clock", 10)
        cls.tf_pub = cls.node.create_publisher(TFMessage, "/tf", 50)
        cls.static_pub = cls.node.create_publisher(
            TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.cloud_pub = cls.node.create_publisher(
            PointCloud2, "/cloud_registered_body", qos_profile_sensor_data)
        cls.seed_pub = cls.node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        cls.pose_sub = cls.node.create_subscription(PoseStamped, "/relocalization/pose", cls.outputs.append, 20)
        cls.tf_sub = cls.node.create_subscription(
            TFMessage, "/tf", cls.record_tf, 100)
        cls.status_sub = cls.node.create_subscription(
            DiagnosticArray, "/relocalization/status", cls.statuses.append, 20)
        cls.map_sub = cls.node.create_subscription(
            PointCloud2, "/relocalization/map_cloud", cls.maps.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.il = transform([-0.011, -0.02329, 0.04412], [0.09, -0.08, 0.13])
        cls.bl = transform([0.07, -0.09, 0.145], [0.11, -0.12, -1.1])
        cls.bi = cls.bl @ np.linalg.inv(cls.il)
        cls.mo = transform([4.0, -3.0, 0.5], [0.04, -0.06, 0.35])
        points = []
        for x in np.linspace(-3, 3, 35):
            for y in np.linspace(-2.4, 2.4, 29):
                points.extend([
                    [x, y, 0.12*math.sin(x)+0.08*math.cos(2*y)],
                    [x, 3+0.1*math.sin(y), y+2.5],
                    [-3.4+0.07*math.cos(x), x, y+2.5]])
        cls.world_points = np.c_[np.array(points), np.ones(len(points))]
        pcd = cls.directory / "map.pcd"
        with pcd.open("w") as stream:
            stream.write(
                f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
                f"TYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n"
                f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
            np.savetxt(stream, np.array(points), fmt="%.8f")
        params = dict(
            use_sim_time=True, prior_pcd_file=str(pcd),
            t_imu_lidar=cls.il[:3, 3].tolist(), R_imu_lidar=cls.il[:3, :3].flatten().tolist(),
            registration_rate=8.0, tf_publish_rate=40.0, num_threads=2,
            map_leaf_size=0.15, scan_leaf_size=0.15, max_scan_age=0.5, max_job_age=1.0,
            degraded_after=0.6)
        config = cls.directory / "params.yaml"
        config.write_text(yaml.safe_dump({"small_gicp_relocalizer": {"ros__parameters": params}}))
        cls.process = subprocess.Popen(
            [args.binary, "--ros-args", "--params-file", str(config)], stdout=cls.log, stderr=cls.log)
        cls.static_pub.publish(TFMessage(transforms=[cls.tf("base_link", "lidar", cls.bl, 0.0)]))
        cls.pump(1.0)
        if cls.process.poll() is not None:
            raise AssertionError("relocalizer exited; see retained node log")
        if not cls.cloud_pub.get_subscription_count():
            raise AssertionError("DDS discovery failed")

    @classmethod
    def cleanup(cls):
        if cls.process:
            cls.process.terminate()
            cls.process.wait(timeout=8)
        cls.log.flush()
        cls.log.seek(0)
        print("Retained test artifacts:", cls.directory)
        print(cls.log.read()[-3000:])
        cls.log.close()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def record_tf(cls, message):
        cls.transforms.extend(message.transforms)
        if cls.tf_buffer is not None:
            for transform in message.transforms:
                cls.tf_buffer.set_transform(transform, "synthetic_test")

    @classmethod
    def ob(cls, seconds):
        elapsed = seconds-1000.0
        return transform([1.4+0.06*elapsed, -0.8+0.02*elapsed, 0.2], [0, 0, -0.25+0.015*elapsed])

    @classmethod
    def tf(cls, parent, child, matrix, seconds):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = stamp(seconds)
        t, q = matrix[:3, 3], quaternion(matrix)
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, t)
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
        return msg

    @classmethod
    def cloud(cls, seconds, frame="body", empty=False, distant=False):
        matrix = cls.bi
        base = cls.mo @ cls.ob(seconds)
        points = (np.linalg.inv(matrix) @ np.linalg.inv(base) @ cls.world_points.T).T[:, :3]
        if distant:
            points += [1000, -1000, 500]
        if empty:
            points = points[:0]
        return point_cloud2.create_cloud_xyz32(
            Header(stamp=stamp(seconds), frame_id=frame), points.tolist())

    @classmethod
    def pump(cls, duration, clouds=True, odom=True, frame="body", empty=False, distant=False, jump=False):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            cls.time += 0.02
            cls.clock_pub.publish(Clock(clock=stamp(cls.time)))
            if odom:
                local = cls.ob(cls.time)
                if jump:
                    local = local.copy()
                    local[0, 3] += 100
                message = cls.tf("odom", "base_link", local, cls.time)
                cls.local_inputs[(message.header.stamp.sec, message.header.stamp.nanosec)] = local
                cls.tf_pub.publish(TFMessage(transforms=[message]))
            if clouds and cls.step % 5 == 0:
                cls.cloud_pub.publish(cls.cloud(cls.time-0.02, frame, empty, distant))
            cls.step += 1
            target = time.monotonic()+0.02
            while time.monotonic() < target:
                rclpy.spin_once(cls.node, timeout_sec=0.002)

    @classmethod
    def seed(cls, matrix=None, frame="map", zero=False, invalid=False):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = frame
        message.header.stamp = Time() if zero else stamp(cls.time)
        matrix = cls.mo @ cls.ob(cls.time) if matrix is None else matrix
        p, q = matrix[:3, 3], quaternion(matrix)
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = map(float, p)
        if invalid:
            message.pose.pose.orientation.w = 0.0
        else:
            message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w = map(float, q)
        cls.seed_pub.publish(message)

    def globals(self):
        return [tf for tf in self.transforms if tf.header.frame_id == "map"]

    def latest_status(self):
        self.assertTrue(self.statuses)
        status = self.statuses[-1].status[0]
        return status.message, {value.key: value.value for value in status.values}

    def wait_pose(self, previous, timeout=2.0):
        deadline = time.monotonic()+timeout
        while len(self.outputs) <= previous and time.monotonic() < deadline:
            self.pump(0.1)
        self.assertGreater(len(self.outputs), previous)

    def assert_correction(self):
        tf = self.globals()[-1]
        msg = PoseStamped()
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        actual = matrix_from_pose(msg.pose)
        np.testing.assert_allclose(actual[:3, 3], self.mo[:3, 3], atol=0.03)
        np.testing.assert_allclose(actual[:3, :3], self.mo[:3, :3], atol=0.007)

    def test_01_wait_init_no_identity_tf_and_invalid_seeds(self):
        self.assertFalse(self.outputs)
        self.assertFalse(self.globals())
        self.assertTrue(self.maps)
        self.assertEqual(self.maps[-1].header.frame_id, "map")
        self.seed(frame="odom")
        self.seed(invalid=True)
        self.pump(0.6)
        self.assertFalse(self.outputs)
        self.assertFalse(self.globals())

    def test_02_init_and_moving_tracking_use_ekf_at_scan_time(self):
        self.seed(zero=True)
        self.wait_pose(0)
        self.pump(0.6)
        self.assert_correction()
        output = self.outputs[-1]
        seconds = output.header.stamp.sec+output.header.stamp.nanosec*1e-9
        np.testing.assert_allclose(matrix_from_pose(output.pose),
                                   self.mo @ self.ob(seconds), atol=0.03)
        state, values = self.latest_status()
        self.assertEqual(state, "TRACKING")
        self.assertGreaterEqual(int(values["confirmation_count"]), 2)
        self.assertLess(float(values["euclidean_rmse_m"]), 0.15)
        for tf in self.globals():
            self.assertEqual(tf.child_frame_id, "odom")

    def test_03_wrong_frame_empty_cloud_and_missing_tf_keep_old_correction(self):
        self.pump(0.35, clouds=False)  # Drain an already valid in-flight scan.
        previous = len(self.outputs)
        self.pump(0.7, frame="lidar")
        self.assertEqual(len(self.outputs), previous)
        self.assert_correction()
        self.pump(0.6, empty=True)
        self.assertEqual(len(self.outputs), previous)
        self.pump(0.6, odom=False)
        self.assertEqual(len(self.outputs), previous)
        self.wait_pose(previous)
        self.pump(0.6)
        self.assert_correction()

    def test_04_bad_reinitialization_preserves_valid_context(self):
        self.pump(0.35, clouds=False)  # Drain results submitted before the new seed.
        previous = len(self.outputs)
        before = len(self.globals())
        bad = self.mo @ self.ob(self.time)
        bad[0, 3] += 100
        self.seed(bad)
        self.pump(0.7)
        self.assertEqual(len(self.outputs), previous)
        self.assertGreater(len(self.globals()), before)
        self.assert_correction()
        self.seed()  # Replace the retained invalid seed with a valid manual request.
        self.wait_pose(previous, timeout=2.5)
        self.pump(0.6)
        self.assert_correction()

    def test_05_new_generation_can_replace_old_map_position(self):
        # Rapid requests must leave only the latest initialization seed active.
        bad = self.mo @ self.ob(self.time)
        bad[1, 3] -= 100
        self.seed(bad)
        type(self).mo = transform([4.6, -3.3, 0.65], [0.03, -0.05, 0.48])
        self.seed()
        previous = len(self.outputs)
        self.wait_pose(previous)
        self.pump(0.6)
        self.assert_correction()

    def test_06_stale_correction_keeps_tf_queries_working_and_recovers(self):
        type(self).tf_buffer = Buffer()
        self.addCleanup(setattr, type(self), "tf_buffer", None)
        self.pump(0.35, clouds=False)  # Finish a scan already in flight.
        previous = len(self.outputs)
        held = self.globals()[-1].transform
        self.pump(1.7, clouds=False)
        count = len(self.globals())
        self.pump(0.5, clouds=False)
        self.assertGreater(len(self.globals()), count)

        # Advance beyond the default 10-second TF cache while odometry keeps moving.
        type(self).time += 12.0
        self.pump(0.6, clouds=False)
        self.assertEqual(len(self.outputs), previous)
        state, values = self.latest_status()
        self.assertEqual(state, "DEGRADED")
        self.assertEqual(values["tf_refreshing"], "true")
        self.assertGreater(float(values["correction_age_s"]), 12.0)
        self.assertEqual(self.globals()[-1].transform, held)
        for target, source in [("map", "odom"), ("odom", "base_link"), ("map", "base_link")]:
            result = self.tf_buffer.lookup_transform(target, source, RosTime())
            seconds = result.header.stamp.sec + result.header.stamp.nanosec * 1e-9
            self.assertLess(abs(self.time - seconds), 0.1)

        self.wait_pose(previous)
        self.pump(0.5)
        self.assert_correction()
        self.assertEqual(self.latest_status()[0], "TRACKING")

    def test_07_graph_and_local_tf_are_untouched(self):
        publishers = self.node.get_publisher_names_and_types_by_node("small_gicp_relocalizer", "/")
        self.assertFalse({"/odom", "/lio/odometry", "/odometry/filtered"} &
                         {name for name, _ in publishers})
        services = self.node.get_service_names_and_types_by_node("small_gicp_relocalizer", "/")
        self.assertFalse(any("set_pose" in name for name, _ in services))
        local = [t for t in self.transforms if t.child_frame_id == "base_link"]
        self.assertGreater(len(local), 50)
        for tf in local:
            self.assertEqual(tf.header.frame_id, "odom")
            key = tf.header.stamp.sec, tf.header.stamp.nanosec
            expected = self.local_inputs[key]
            np.testing.assert_allclose(
                [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z],
                expected[:3, 3], atol=1e-12)

    def test_08_time_backwards_invalidates_and_requires_new_seed(self):
        type(self).time -= 100
        self.pump(0.6)
        count = len(self.globals())
        previous = len(self.outputs)
        self.pump(0.6)
        self.assertEqual(len(self.globals()), count)
        self.assertEqual(len(self.outputs), previous)
        self.seed()
        self.wait_pose(previous)
        self.pump(0.5)
        self.assert_correction()

    def test_09_odom_discontinuity_invalidates_correction(self):
        self.pump(0.6, jump=True)
        count, previous = len(self.globals()), len(self.outputs)
        self.pump(0.6, jump=True)
        self.assertEqual(len(self.globals()), count)
        self.assertEqual(len(self.outputs), previous)
        self.assertEqual(self.latest_status()[0], "WAIT_INIT")

    def test_10_exit_leaves_local_tf_running(self):
        self.process.terminate()
        self.process.wait(timeout=8)
        local = len([t for t in self.transforms if t.child_frame_id == "base_link"])
        self.pump(0.5, clouds=False)
        self.assertGreater(len([t for t in self.transforms if t.child_frame_id == "base_link"]), local)


if __name__ == "__main__":
    unittest.main(argv=[__file__]+remaining)

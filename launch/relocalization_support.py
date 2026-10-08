"""Shared standalone/bringup construction; mounting values come from static TF."""
import math
import os

import yaml
from launch_ros.actions import Node


def read_fast_lio_parameters(path):
    with open(path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise RuntimeError("FAST-LIO config must be a ROS parameter mapping")

    def flatten(data, prefix=""):
        result = {}
        for key, value in data.items():
            name = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                result.update(flatten(value, name))
            else:
                result[name] = value
        return result

    parameters = {}
    for scope in ("/**", "laser_mapping", "/laser_mapping"):
        block = document.get(scope, {}).get("ros__parameters", {})
        if not isinstance(block, dict):
            raise RuntimeError("Invalid FAST-LIO ros__parameters block")
        parameters.update(flatten(block))
    return parameters


def read_imu_extrinsics(path):
    parameters = read_fast_lio_parameters(path)
    try:
        translation = [float(v) for v in parameters["mapping.extrinsic_T"]]
        rotation = [float(v) for v in parameters["mapping.extrinsic_R"]]
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("FAST-LIO config requires mapping.extrinsic_T/R") from exc
    if len(translation) != 3 or len(rotation) != 9 or not all(
            math.isfinite(v) for v in translation + rotation):
        raise RuntimeError("Invalid IMU-lidar extrinsic arrays")
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[3*k+i]*rotation[3*k+j] for k in range(3))
            if abs(dot-float(i == j)) > 1e-6:
                raise RuntimeError("extrinsic_R must be orthonormal")
    r = rotation
    determinant = (r[0]*(r[4]*r[8]-r[5]*r[7])
                   - r[1]*(r[3]*r[8]-r[5]*r[6])
                   + r[2]*(r[3]*r[7]-r[4]*r[6]))
    if abs(determinant-1.0) > 1e-6:
        raise RuntimeError("extrinsic_R must be right handed")
    return translation, rotation


def create_relocalizer(prior_pcd_file, relocalization_config, fast_lio_config, use_sim_time):
    prior_pcd_file = os.path.expanduser(prior_pcd_file)
    if (not os.path.isabs(prior_pcd_file)
            or not prior_pcd_file.lower().endswith(".pcd")
            or not os.path.isfile(prior_pcd_file)):
        raise RuntimeError("重定位要求 prior_pcd_file 为已有 PCD 地图的绝对路径")
    if not os.path.isfile(relocalization_config):
        raise RuntimeError(f"重定位配置不存在：{relocalization_config}")
    translation, rotation = read_imu_extrinsics(fast_lio_config)
    parameters = read_fast_lio_parameters(fast_lio_config)
    if (not parameters.get("publish.scan_publish_en", True)
            or not parameters.get("publish.scan_bodyframe_pub_en", True)):
        raise RuntimeError("重定位需要 FAST-LIO 开启 scan_publish_en 和 scan_bodyframe_pub_en")
    # bringup forces mapping.extrinsic_est_en=false. Standalone users must
    # attach to that same fixed-extrinsic LIO instance; see README.
    return Node(
        package="scout_mini_relocalization", executable="small_gicp_relocalizer",
        name="small_gicp_relocalizer", output="screen",
        parameters=[relocalization_config, {
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "t_imu_lidar": translation,
            "R_imu_lidar": rotation,
        }],
    )

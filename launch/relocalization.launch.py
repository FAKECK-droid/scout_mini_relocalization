"""Attach localization to an already running FAST-LIO/EKF chain."""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def _create(context):
    share = get_package_share_directory("scout_mini_relocalization")
    spec = importlib.util.spec_from_file_location(
        "relocalization_support", os.path.join(share, "launch", "relocalization_support.py"))
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    return [support.create_relocalizer(
        LaunchConfiguration("prior_pcd_file").perform(context),
        LaunchConfiguration("relocalization_config").perform(context),
        LaunchConfiguration("fast_lio_config").perform(context),
        ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
    )]


def generate_launch_description():
    share = get_package_share_directory("scout_mini_relocalization")
    fast_lio_share = get_package_share_directory("fast_lio")
    return LaunchDescription([
        DeclareLaunchArgument("prior_pcd_file", description="Existing native-frame PCD absolute path"),
        DeclareLaunchArgument("relocalization_config",
                              default_value=os.path.join(share, "config", "relocalization.yaml")),
        DeclareLaunchArgument("fast_lio_config",
                              default_value=os.path.join(fast_lio_share, "config", "mid360_jetson.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=_create),
    ])

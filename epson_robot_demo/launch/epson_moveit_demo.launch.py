""" Launch file for running the Epson MoveIt  demo program"""

#  Copyright 2025 Seiko Epson Corporation
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

import os
import yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription


def load_yaml(package_name, file_path):
    """Function for loading a yaml file."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r', encoding="utf-8") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None



def generate_launch_description():
    """Launch Description generator function."""
    # Set specific arguments -----------------------------------------------------------

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("rb_model", default_value="",  description="Epson robot model name")
    )

    declared_arguments.append(
        DeclareLaunchArgument("acc_factor", default_value="1.0",
                              description="Factor of acceleration.")
    )

    declared_arguments.append(
        DeclareLaunchArgument("vel_factor", default_value="1.0",
                              description="Factor of velocity.")
    )

    declared_arguments.append(
        DeclareLaunchArgument('number_cycles', default_value='3',
                              description='Number of times to repeat the pick-and-place cycle.'))

    lc_rb_model = LaunchConfiguration("rb_model")
    lc_acc_factor = LaunchConfiguration("acc_factor")
    lc_vel_factor = LaunchConfiguration("vel_factor")
    lc_number_cycles = LaunchConfiguration('number_cycles')

    description_folder = "epson_robot_description"
    moveit_config_folder = "epson_robot_moveit_config"
    xacro_file = "epson_robot.urdf.xacro"
    srdf_file = "epson_robot.srdf.xacro"

    demo_moveit_config_yaml_file_path = (
        get_package_share_directory('epson_robot_demo')
        + '/config/epson_moveit_demo.yaml')

    moveit_simple_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare(moveit_config_folder), 'models',
            lc_rb_model, "config/moveit_controllers.yaml"])

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    demo_moveit_params = {
        'rb_model': lc_rb_model,
        'acc_factor': lc_acc_factor,
        'vel_factor': lc_vel_factor,
        'number_cycles': lc_number_cycles,
    }

    robot_description_config = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
         PathJoinSubstitution([FindPackageShare(description_folder),
                               "models", lc_rb_model, "urdf", xacro_file]), ' ',
         'model:=', lc_rb_model, ' '])
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = launch_ros.descriptions.ParameterValue(
        Command(
            [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
             PathJoinSubstitution([FindPackageShare(moveit_config_folder),
                                   "models", lc_rb_model, 'srdf', srdf_file]), ' ']
        ), value_type=str)
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    robot_description_kinematics = {"robot_description_kinematics": load_yaml(
        moveit_config_folder, "config/kinematics.yaml")
    }

    joint_limits_yaml = PathJoinSubstitution(
        [FindPackageShare(moveit_config_folder), 'models',
         lc_rb_model, "config/joint_limits.yaml"])

    request_adpters = """default_planner_request_adapters/AddTimeOptimalParameterization \
    default_planner_request_adapters/ResolveConstraintFrames \
    default_planner_request_adapters/FixWorkspaceBounds \
    default_planner_request_adapters/FixStartStateBounds \
    default_planner_request_adapters/FixStartStateCollision \
    default_planner_request_adapters/FixStartStatePathConstraints"""

    resample_dt = 0.002
    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl", "chomp"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": request_adpters,
            "start_state_max_bounds_error": 0.1,
            "resample_dt": resample_dt,
        },
        "chomp": {
            "planning_plugin": "chomp_interface/CHOMPPlanner",
            "request_adapters": request_adpters,
            "start_state_max_bounds_error": 0.1,
            "resample_dt": resample_dt,
        }
    }

    ompl_planning_yaml = load_yaml(
        moveit_config_folder, "config/ompl_planning.yaml"
    )
    planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    chomp_planning_yaml = load_yaml(
        moveit_config_folder, "config/chomp_planning.yaml"
    )
    planning_pipeline_config["chomp"].update(chomp_planning_yaml)

    demo_moveit_pickandplace_node = Node(
        package='epson_robot_demo',
        executable='epson_moveit_demo',
        output='screen',
        parameters=[
            demo_moveit_params,
            demo_moveit_config_yaml_file_path,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
            joint_limits_yaml,
            moveit_controllers
        ],)

    return LaunchDescription(declared_arguments + [demo_moveit_pickandplace_node, ])

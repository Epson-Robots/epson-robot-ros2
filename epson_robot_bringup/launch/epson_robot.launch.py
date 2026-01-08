"""
Launch file for starting the EPSON ROS2 node
"""

#  Copyright 2025 Seiko Epson Corporation

#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

import os
import yaml
import launch_ros.descriptions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition
from launch import LaunchDescription

def load_yaml(package_name, file_path):
    """ Function for loading a yaml file. """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None




def generate_launch_description():
    """Launch Description generator function. """

    declared_arguments=[]

    declared_arguments.append(
        DeclareLaunchArgument("use_fake_hardware",
                              default_value="false",
                              description="Testing is possible without actual hardware")
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz")
    )

    declared_arguments.append(
        DeclareLaunchArgument("controller_ip",
                              default_value="192.168.0.1",
                              description="IP address of epson robot controller")
    )

    declared_arguments.append(
        DeclareLaunchArgument("controller_port",
                               default_value="7000",
                               description="Port number for built-in messages on epson robot controller")
    )

    declared_arguments.append(
        DeclareLaunchArgument("security",
                              default_value="false",
                              description="Secure communication between epson robot controller and the Linux PC")
    )

    declared_arguments.append(
        DeclareLaunchArgument("client_ip",
                              default_value="",
                              description="IP address of the Linux PC")
    )

    declared_arguments.append(
        DeclareLaunchArgument("client_builtinmsg_port",
                              default_value="0",
                              description="Port number for built-in messages on the Linux PC")
    )

    declared_arguments.append(
        DeclareLaunchArgument("client_userdata_port",
                              default_value="0",
                              description="Port number for user data on Linux PC")
    )

    declared_arguments.append(
        DeclareLaunchArgument("send_format",
                              default_value="0",
                              description="In case 1, it is possible to control standard I/O output during robot operation")
    )

    declared_arguments.append(
        DeclareLaunchArgument("recv_format",
                              default_value="0",
                              description="In case 1, it is possible to receive standard I/O input during robot operation")
    )

    declared_arguments.append(
        DeclareLaunchArgument("rb_model", default_value="",  description="Epson robot model name")
    )

    declared_arguments.append(
        DeclareLaunchArgument("weight",
                              default_value="Default",
                              description="The mass of the gripper attached to epson robot")
    )

    declared_arguments.append(
        DeclareLaunchArgument("inertia",
                              default_value="Default",
                              description="The inertia of the gripper attached to epson robot")
    )

    declared_arguments.append(
        DeclareLaunchArgument("eccentricity",
                               default_value="Default",
                               description="The eccentricity between of the tool center point and the gripeer center of mass")
    )

    declared_arguments.append(
        DeclareLaunchArgument("ca_cert",
                              default_value="",
                              description="Certificate path of the CA-signed certificate")
    )

    declared_arguments.append(
        DeclareLaunchArgument("client_cert",
                              default_value="",
                              description="CA-signed certificate for a Linux PC application")
    )

    declared_arguments.append(
        DeclareLaunchArgument("key",
                              default_value="",
                              description="Private key for a Linux PC application")
    )

    declared_arguments.append(
        DeclareLaunchArgument("governance",
                              default_value="",
                              description="Governance document for a Linux PC application signed by a CA")
    )

    declared_arguments.append(
        DeclareLaunchArgument("permissions",
                              default_value="",
                              description="Permissions document for a Linux PC application signed by a CA")
    )

    declared_arguments.append(
        DeclareLaunchArgument("password",
                              default_value="",
                              description="Password for RT Motion Control.")
    )

    declared_arguments.append(
        DeclareLaunchArgument("buffer_size",
                              default_value="10",
                              description="Buffer size allocated for motion commands")
    )

    declared_arguments.append(
        DeclareLaunchArgument("namespace",
                              default_value="epson_robot_control",
                              description="Prefix for service and topic message. Node name for service")
    )

    declared_arguments.append(
        DeclareLaunchArgument("log_level",
                              default_value="2",
                              choices=['0', '1', '2', '3', '4', '5'],
                              description="0:UNSET 1:DEBUG 2:INFO 3:WARN 4:ERROR 5:FATAL.")
    )

    lc_use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    lc_launch_rviz       = LaunchConfiguration("launch_rviz")
    lc_controller_ip       = LaunchConfiguration("controller_ip")
    lc_controller_port       = LaunchConfiguration("controller_port")
    lc_client_ip       = LaunchConfiguration("client_ip")
    lc_client_builtinmsg_port      = LaunchConfiguration("client_builtinmsg_port")
    lc_client_userdata_port      = LaunchConfiguration("client_userdata_port")
    lc_send_format     = LaunchConfiguration("send_format")
    lc_recv_format     = LaunchConfiguration("recv_format")
    lc_rb_model = LaunchConfiguration("rb_model")
    lc_weight = LaunchConfiguration("weight")
    lc_inertia = LaunchConfiguration("inertia")
    lc_eccentricity = LaunchConfiguration("eccentricity")
    lc_security = LaunchConfiguration("security")
    lc_ca_cert = LaunchConfiguration("ca_cert")
    lc_client_cert = LaunchConfiguration("client_cert")
    lc_key = LaunchConfiguration("key")
    lc_governance = LaunchConfiguration("governance")
    lc_permissions = LaunchConfiguration("permissions")
    lc_password = LaunchConfiguration("password")
    lc_buffer_size = LaunchConfiguration("buffer_size")
    lc_namespace = LaunchConfiguration("namespace")
    lc_log_level = LaunchConfiguration("log_level")

    # Sets parameters  ------------------------------------------------------------------------
    description_folder = "epson_robot_description"
    moveit_config_folder = "epson_robot_moveit_config"
    xacro_file = "epson_robot.urdf.xacro"
    srdf_file = "epson_robot.srdf.xacro"

    robot_description_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([FindPackageShare(description_folder),
                                  "models", lc_rb_model,
                                  "urdf", xacro_file]), ' ',
            'use_fake_hardware:=', lc_use_fake_hardware, ' ',
            'controller_ip:=', lc_controller_ip, ' ',
            'controller_port:=', lc_controller_port, ' ',
            'client_ip:=', lc_client_ip, ' ',
            'client_builtinmsg_port:=', lc_client_builtinmsg_port, ' ',
            'client_userdata_port:=', lc_client_userdata_port, ' ',
            'send_format:=', lc_send_format, ' ',
            'recv_format:=', lc_recv_format, ' ',
            'recv_format:=', lc_recv_format, ' ',
            'rb_model:=', lc_rb_model, ' ',
            'weight:=', lc_weight, ' ',
            'inertia:=', lc_inertia, ' ',
            'eccentricity:=', lc_eccentricity, ' ',
            'security:=', lc_security, ' ',
            'ca_cert:=', lc_ca_cert, ' ',
            'client_cert:=', lc_client_cert, ' ',
            'key:=', lc_key, ' ',
            'governance:=', lc_governance, ' ',
            'permissions:=', lc_permissions, ' ',
            'password:=', lc_password, ' ',
            'buffer_size:=', lc_buffer_size, ' ',
            'namespace:=', lc_namespace, ' ',
            'log_level:=', lc_log_level, ' ',
        ])
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config =  launch_ros.descriptions.ParameterValue( Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([FindPackageShare(moveit_config_folder),
                                  "models", lc_rb_model,
                                  'srdf', srdf_file]), ' '
        ]) , value_type=str)
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    kinematics_yaml = {"robot_description_kinematics": load_yaml(
    moveit_config_folder, "config/kinematics.yaml")
    }

    joint_limits_yaml = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_folder), 'models',
            lc_rb_model, "config/joint_limits.yaml"
    ])

    request_adpters = """default_planner_request_adapters/AddTimeOptimalParameterization \
    default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds \
    default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision \
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

    moveit_controllers = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_folder), 'models',
            lc_rb_model, "config/moveit_controllers.yaml"
    ])

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_planning_scene_hz": 30.0,
    }

    # -------------------------------------------------------------------

    # Start the actual move_group node/action server --------------------
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            planning_pipeline_config,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_folder), 'models', lc_rb_model, 'rviz', 'view_robot.rviz'
        ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(lc_launch_rviz),
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_folder), 'models',
            lc_rb_model, "config/epson_robot_controllers.yaml"
        ])
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

     # launch joint state controller
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["epson_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # launch joint trajectory controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["epson_joint_trajectory_controller", "-c", "/controller_manager"],
    )
    return LaunchDescription(
        declared_arguments
        + [
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner ,
            robot_controller_spawner ,
        ]
    )
    # --------------------------------------------------------------------

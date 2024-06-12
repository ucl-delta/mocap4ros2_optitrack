# Copyright 2021 Institute for Robotics and Intelligent Machines,
#                Georgia Institute of Technology
# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Christian Llanes <christian.llanes@gatech.edu>
# Author: David Vargas Frutos <david.vargas@urjc.es>

import os

from ament_index_python.packages import get_package_share_directory
import launch

from launch import LaunchDescription
from launch.actions import EmitEvent, DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable, OpaqueFunction
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import lifecycle_msgs.msg

def get_node(context):
    """ Returns the follow_path behavior node """
    config_file = LaunchConfiguration('config_file').perform(context)

    print(config_file)
    driver_node = LifecycleNode(
        name='mocap4r2_optitrack_driver_node',
        namespace=LaunchConfiguration("namespace").perform(context),
        package='mocap4r2_optitrack_driver',
        executable='mocap4r2_optitrack_driver_main',
        output='screen',
        parameters=[config_file],
    )

    # Make the driver node take the 'configure' transition
    driver_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the driver node take the 'activate' transition
    driver_activate_trans_event = EmitEvent(
       event = ChangeState(
           lifecycle_node_matcher = launch.events.matchers.matches_action(driver_node),
           transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ),
        condition=LaunchConfigurationEquals("auto_activate", "true")
    )

    return [
        driver_node,
        driver_configure_trans_event,
        driver_activate_trans_event
    ]


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')


    namespace_launch_arg = DeclareLaunchArgument("namespace")

    default_mocap_params = os.path.join(
            get_package_share_directory('mocap4r2_optitrack_driver'),
            'config', 'mocap4r2_optitrack_driver_params.yaml'
        )
    print(default_mocap_params)
    config_file_launch_arg = DeclareLaunchArgument('config_file', default_value=default_mocap_params)

    auto_activate_launch_arg = DeclareLaunchArgument("auto_activate", 
                                                     description="Automatically activate node",
                                                     default_value="true")

    # Create the launch description and populate
    ld = LaunchDescription()
    
    ld.add_action(auto_activate_launch_arg)
    ld.add_action(namespace_launch_arg)
    ld.add_action(config_file_launch_arg)
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(OpaqueFunction(function=get_node))

    return ld

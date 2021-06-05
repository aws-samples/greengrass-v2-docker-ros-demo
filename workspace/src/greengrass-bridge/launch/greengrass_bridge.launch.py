#!/usr/bin/env python
"""
Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
SPDX-License-Identifier: MIT-0

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ros_topics = LaunchConfiguration('ros_topics')
    iot_topics = LaunchConfiguration('iot_topics')
    
    return LaunchDescription([
        DeclareLaunchArgument(
          'ros_topics',
           default_value='[]',
           description='List of ROS2 topics to bridge.'),
        DeclareLaunchArgument(
          'iot_topics',
           default_value='[]',
           description='List of IoT topics to bridge.'),
        Node(
             package='greengrass_bridge', executable='greengrass_bridge', output='screen',
             parameters=[
                {"ros_topics": ros_topics},
                {"iot_topics": iot_topics}
             ],
             name=['greengrass_bridge'])
    ])

if __name__ == '__main__':
    generate_launch_description()
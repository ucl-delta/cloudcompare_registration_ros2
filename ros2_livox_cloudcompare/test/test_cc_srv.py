# Copyright 2019 Apex.AI, Inc.
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

import time
import unittest
import os

import launch
from launch.launch_service import LaunchService
import launch_ros
import launch_ros.actions
import launch_testing.actions
from launch_testing.io_handler import ActiveIoHandler

from ament_index_python.packages import get_package_share_directory

import pytest

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

from pointcloud_registration_msgs.srv import Registration
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point

@pytest.mark.rostest
def generate_test_description():
    # Normally, talker publishes on the 'chatter' topic and listener listens on the
    # 'chatter' topic, but we want to show how to use remappings to munge the data so we
    # will remap these topics when we launch the nodes and insert our own node that can
    # change the data as it passes through

    cc_node = launch_ros.actions.Node(
            package='ros2_livox_cloudcompare',
            executable='ros2_livox_cloudcompare',
            name='cloudcompare',
            additional_env={'PYTHONUNBUFFERED': '1'},
            # parameters=[
            #     {"drill_configuration_file": os.path.join(get_package_share_directory("drilling_application"), "config", "uEyeRef.txt")},
            #     {"template_configuration_file": os.path.join(get_package_share_directory("drilling_application"), "config", "jig_holes.txt")}
            # ]
    )
    
    
    return (
        launch.LaunchDescription([
            cc_node,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'cc_node': cc_node
        }
    )

class TestCCService(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_cc_node')

    def tearDown(self):
        self.node.destroy_node()

    def create_random_pointcloud2(self, num_points=100, frame_id="map"):
        # Generate random points
        points = np.random.rand(num_points, 3) * 10  # Random points in a 10x10x10 cube

        # Create a header
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = frame_id

        # Define the fields for PointCloud2 (x, y, z)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Create the PointCloud2 message
        pointcloud2_msg = pc2.create_cloud(header, fields, points)

        return pointcloud2_msg

    def test_cc_send_request1(self,
                            launch_service: LaunchService,
                            cc_node: Node,
                            proc_output: ActiveIoHandler
                            ):
        
        # Create a client for the service
        client = self.node.create_client(Registration, 'register')

        # Wait for the service to be available
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail("Service 'register' not available.")

        # Create a request object
        request = Registration.Request()

        # Create a Random Pointcloud
        request.name = "test_pointcloud"
        request.pointcloud = self.create_random_pointcloud2(frame_id="lidar")
        request.initial_transform.header.frame_id = "map"
        request.initial_transform.child_frame_id = "lidar"
        request.clip_distance_threshold = 0.3

        try:

            # Send the request
            future = client.call_async(request)

            # Wait for the result
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

            # Test succeeds if a response is received
            if future.result() is not None:
                response = future.result()
                print("Received response:", response)
                self.assertEqual(
                    response.transform.header.frame_id,
                    request.initial_transform.header.frame_id, 
                    "Frame IDs do not match")
                self.assertEqual(
                    response.transform.child_frame_id,
                    request.initial_transform.child_frame_id,
                    "Child Frame IDs do not match")
            else:
                self.fail("Failed to receive a response.")
        
        finally:
            self.node.destroy_client(client)
    
    def test_cc_send_request2(self,
                            launch_service: LaunchService,
                            cc_node: Node,
                            proc_output: ActiveIoHandler
                            ):
        
        # Create a client for the service
        client = self.node.create_client(Registration, 'register')

        # Wait for the service to be available
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail("Service 'register' not available.")

        # Create a request object
        request = Registration.Request()

        # Create a Random Pointcloud
        request.name = "test_pointcloud"
        request.pointcloud = self.create_random_pointcloud2(frame_id="lidar")
        request.initial_transform.header.frame_id = "map"
        request.initial_transform.child_frame_id = "lidar"
        request.clip_distance_threshold = 0.3

        for i in [-1.0, 1.0]:
            spoint = Point()
            spoint.x = i * 5.0
            spoint.y = i * 5.0
            spoint.z = i * 5.0
            request.crop_corner.append(spoint)

        try:

            # Send the request
            future = client.call_async(request)

            # Wait for the result
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

            # Test succeeds if a response is received
            if future.result() is not None:
                response = future.result()
                print("Received response:", response)
                self.assertEqual(
                    response.transform.header.frame_id,
                    request.initial_transform.header.frame_id, 
                    "Frame IDs do not match")
                self.assertEqual(
                    response.transform.child_frame_id,
                    request.initial_transform.child_frame_id,
                    "Child Frame IDs do not match")
            else:
                self.fail("Failed to receive a response.")
        
        finally:
            self.node.destroy_client(client)
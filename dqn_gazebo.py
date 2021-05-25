#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Ryan Shim, Gilbert
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Empty
from my_msgs.srv import Goal, Location
from geometry_msgs.msg import Pose
import sys


# A node to generate goals.
class GazeboInterface(Node):
    def __init__(self, task_number):
        super().__init__('gazebo_interface')
        self.target_position = None

        # Read the 'Goal' Entity Model
        self.entity_name = 'Goal'
        self.entity = open('./goal_box/model.sdf', 'r').read()

        # Initial entity(Goal) position
        self.entity_pose_x = 0.5
        self.entity_pose_y = 0.0

        # Parse task chosen as correct type.
        self.task_number = int(task_number)
        self.goal_list = [
            [1.7, 1.7],
            [-1.7, -1.7],
            [1.7, -1.7],
            [-1.7, 1.7],
            [0.0, 0.0]
        ]

        # Set invoke flag so services are only called once.
        self.invoked = False

        # Initialize clients
        self.delete_entity_client = self.create_client(
            DeleteEntity,
            'delete_entity')
        self.spawn_entity_client = self.create_client(
            SpawnEntity,
            'spawn_entity')
        self.reset_simulation_client = self.create_client(
            Empty,
            'reset_simulation')

        # Initialize services
        self.callback_group = MutuallyExclusiveCallbackGroup()

        if self.task_number == 1:
            self.initialise_services()

        if self.task_number == 2:
            self.location_service = self.create_service(
                Location,
                'person_location',
                self.location_callback)

            self.get_logger().info('Initialised Location Service.')
            self.get_logger().info(
                'Listening for location to invoke DQN gazebo services.')

    def reset_simulation(self):
        reset_req = Empty.Request()

        # check connection to the service server
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'service for reset_simulation is not available, waiting ...')

        self.reset_simulation_client.call_async(reset_req)

    def delete_entity(self):
        delete_req = DeleteEntity.Request()
        delete_req.name = self.entity_name

        # check connection to the service server
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'service for delete_entity is not available, waiting ...')

        future = self.delete_entity_client.call_async(delete_req)
        rclpy.spin_until_future_complete(self, future)

    def spawn_entity(self):
        entity_pose = Pose()
        entity_pose.position.x = self.entity_pose_x
        entity_pose.position.y = self.entity_pose_y

        spawn_req = SpawnEntity.Request()
        spawn_req.name = self.entity_name
        spawn_req.xml = self.entity
        spawn_req.initial_pose = entity_pose

        # check connection to the service server
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'service for spawn_entity is not available, waiting ...')

        future = self.spawn_entity_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future)

    def initialise_services(self):
        """
        This method is only intended to be called once.
        """
        self.invoked = True
        self.get_logger().info('Invoking DQN gazebo services.')
        self.initialize_env_service = self.create_service(
            Goal,
            'initialize_env',
            self.initialize_env_callback,
            callback_group=self.callback_group)

        self.task_succeed_service = self.create_service(
            Goal,
            'task_succeed',
            self.task_succeed_callback,
            callback_group=self.callback_group)

        self.task_failed_service = self.create_service(
            Goal,
            'task_failed',
            self.task_failed_callback,
            callback_group=self.callback_group)

    def location_callback(self, request, response):

        # Create location list object.
        location = [round(request.location_x, 2), round(request.location_y, 2)]
        self.get_logger().info(
            'Request received. Location of person: {}.'.format(location))

        # Check if services have been invoked.
        if not self.invoked:
            self.initialise_services()
            self.entity_pose_x, self.entity_pose_y = location
        return response

    def task_succeed_callback(self, request, response):
        self.delete_entity()
        if self.task_number == 1:
            self.generate_goal_pose()

        self.spawn_entity()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        print('A new goal generated.')
        return response

    def task_failed_callback(self, request, response):
        self.delete_entity()
        self.reset_simulation()
        if self.task_number == 1:
            self.generate_goal_pose()

        self.spawn_entity()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        print('Environment reset')
        return response

    def initialize_env_callback(self, request, response):
        self.delete_entity()
        self.reset_simulation()
        self.spawn_entity()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        print('Environment initialized')
        return response

    def generate_goal_pose(self):
        if self.entity_pose_x == 0.5 and self.entity_pose_y == 0.0:
            index = 0
        else:
            index = self.goal_list.index([
                self.entity_pose_x,
                self.entity_pose_y])

            if index == (len(self.goal_list) - 1):
                index = 0
            else:
                index += 1

        self.entity_pose_x = self.goal_list[index][0]
        self.entity_pose_y = self.goal_list[index][1]


def main(args=sys.argv[1]):
    rclpy.init(args=args)
    gazebo_interface = GazeboInterface(args)
    while True:
        rclpy.spin_once(gazebo_interface, timeout_sec=0.1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

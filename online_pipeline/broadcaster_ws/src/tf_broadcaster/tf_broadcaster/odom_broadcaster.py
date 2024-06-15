# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import math

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.on_timer)


    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'base_link'
        to_frame_rel = 'odom'

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'ti_mmwave_0'
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        self.tf_broadcaster.sendTransform(t)

                    

        
        

    # def borrar_handle(self, msg):
    #     t = TransformStamped()

    #     # Read message content and assign it to
    #     # corresponding tf variables
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = 'world'
    #     t.child_frame_id = self.turtlename

    #     # Turtle only exists in 2D, thus we get x and y translation
    #     # coordinates from the message and set the z coordinate to 0
    #     t.transform.translation.x = msg.x
    #     t.transform.translation.y = msg.y
    #     t.transform.translation.z = 0.0

    #     # For the same reason, turtle can only rotate around one axis
    #     # and this why we set rotation in x and y to 0 and obtain
    #     # rotation in z axis from the message
    #     q = quaternion_from_euler(0, 0, msg.theta)
    #     t.transform.rotation.x = q[0]
    #     t.transform.rotation.y = q[1]
    #     t.transform.rotation.z = q[2]
    #     t.transform.rotation.w = q[3]

    #     # Send the transformation
    #     self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

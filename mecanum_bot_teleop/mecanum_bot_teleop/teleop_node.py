#! /usr/bin/env python3

import sys
import serial
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import Joy


class Teleoperator(Node):
    def __init__(self):
        super().__init__('teleoperator_node')
        self.get_logger().info('Hello I am teleoperator node')

        self.axes = []
        self.buttons = []

        self.arduino_port = '/dev/ttyUSB0'
        if (self.declare_parameter('arduino_port').value):
            self.arduino_port = self.get_parameter('arduino_port').value
        self.get_logger().info(f"Setting arduino_port: {self.arduino_port}")

        self.enable_button = 5
        if (self.declare_parameter('enable_button').value):
            self.arduino_port = self.get_parameter('enable_button').value
        self.get_logger().info(f"Setting enable_button: {self.enable_button}")

        self.create_subscription(
            Joy,
            '/joy',
            self.joy_cb,
            qos_profile=qos_profile_system_default)

    def joy_cb(self, msg):
        self.display_msg(msg)
        self.axes = msg.axes
        self.buttons = msg.buttons

        if (self.buttons[self.enable_button] == 0):
              return

        result = self.compute_velocities(self.axes, self.buttons)

        encoded_data = str(255*result[0]).encode()
        print(f"Sending encoded data: {encoded_data}")
    
    def display_msg(self, msg):
        print("Axes values: ")
        for axis in msg.axes:
            print(f"  -{axis}")
        print("Button values: ")
        for button in msg.buttons:
            print(f"  -{button}")
        print("------------------------")

    def compute_velocities(self, axes, buttons):
        return [axes[0], axes[1], axes[2], axes[3]]


def main(argv=sys.argv):
    rclpy.init(args=argv)

    n = Teleoperator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
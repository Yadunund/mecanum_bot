#! /usr/bin/env python3

import sys
import serial
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import Joy

from mecanum_bot_controller.controller import compute_motor_velocities
from mecanum_bot_controller.controller import Robot

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

        self.serial_connected = False
        try:
            self.ser = serial.Serial(self.arduino_port, 115200)
            self.serial_connected = True
        except:
            print(f"Unable to open connection on serial port:{self.arduino_port}")
        
        wheel_base = 0.3
        if (self.declare_parameter('wheel_base').value):
            self.wheel_base = self.get_parameter('wheel_base').value
        self.get_logger().info(f"Setting wheel base: {wheel_base}")

        track_width = 0.3
        if (self.declare_parameter('track_width').value):
            self.wheel_base = self.get_parameter('track_width').value
        self.get_logger().info(f"Setting track width: {track_width}")

        wheel_radius = 0.05
        if (self.declare_parameter('wheel_radius').value):
            self.wheel_base = self.get_parameter('wheel_radius').value
        self.get_logger().info(f"Setting wheel radius: {wheel_radius}")

        self.robot = Robot(wheel_base, track_width, wheel_radius)

    def joy_cb(self, msg):
        # self.display_msg(msg)
        self.axes = msg.axes
        self.buttons = msg.buttons

        if (self.buttons[self.enable_button] == 0):
              return

        result = compute_motor_velocities(self.axes, self.robot)

        encoded_data = str(result).encode()
        self.get_logger().info(f"Sending encoded data: {encoded_data}")
        if self.serial_connected:
            self.ser.write(encoded_data)
        sleep(0.025)

    def display_msg(self, msg):
        print("Axes values: ")
        for axis in msg.axes:
            print(f"  -{axis}")
        print("Button values: ")
        for button in msg.buttons:
            print(f"  -{button}")
        print("------------------------")

def main(argv=sys.argv):
    rclpy.init(args=argv)

    n = Teleoperator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

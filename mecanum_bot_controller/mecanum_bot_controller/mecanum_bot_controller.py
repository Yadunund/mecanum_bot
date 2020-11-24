#! /usr/bin/env python3

import sys
import serial
import threading
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import Twist

from mecanum_bot_controller.robot import compute_motor_velocities
from mecanum_bot_controller.robot import Robot

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller_node')
        self.get_logger().info('Hello I am base controller node')

        self.axes = []
        self.buttons = []

        self.arduino_port = '/dev/ttyUSB0'
        if (self.declare_parameter('arduino_port').value):
            self.arduino_port = self.get_parameter('arduino_port').value
        self.get_logger().info(f"Setting arduino_port: {self.arduino_port}")

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
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

        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_yaw = 0.0

        self._lock = threading.Lock()
        self.thread = threading.Thread(target=self.send_velocities)
        self.thread.start()

    def cmd_vel_cb(self, msg):
        with self._lock:
            self.desired_x = msg.linear.x
            self.desired_y = msg.linear.y
            self.desired_yaw = msg.angular.z

    def send_velocities(self):
        while True:
            input = [self.desired_x, self.desired_y, self.desired_yaw]
            result = compute_motor_velocities(input, self.robot)
            print(f"Computed wheel velocities: {result}")
            if (self.serial_connected):
                try:
                    encoded_data = str(result).encode()
                    self.get_logger().info(f"Sending encoded data: {encoded_data}")
                    self.ser.write(encoded_data)
                except:
                    print(f"Unable to write wheel velocities to serial port")
                    self.serial_connected = False
            sleep(0.025)


def main(argv=sys.argv):
    rclpy.init(args=argv)

    n = BaseController()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

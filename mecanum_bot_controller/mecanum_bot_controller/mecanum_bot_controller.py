#! /usr/bin/env python3

import sys
import serial
import threading
from time import sleep
import numpy as np

import tf2_ros

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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

        self.odom_pub = self.create_publisher(
                Odometry,
                "/odom",
                qos_profile=qos_profile_services_default)
        
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self, qos_profile_services_default)

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

        max_v = 0.8
        max_w = 1.0

        self.robot = Robot(wheel_base, track_width, wheel_radius, max_v, max_w)

        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_yaw = 0.0
        
        # book keeping velocities and positions
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_odom_publish_time = 0.0

        self._lock = threading.Lock()
        self.thread = threading.Thread(target=self.send_velocities)
        self.thread.start()

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def cmd_vel_cb(self, msg):
        with self._lock:
            self.desired_x = msg.linear.x
            self.desired_y = msg.linear.y
            self.desired_yaw = msg.angular.z

            self.vx = msg.linear.x * self.robot.max_v
            self.vy = msg.linear.y * self.robot.max_v
            self.vyaw = msg.angular.z * self.robot.max_w

    def send_velocities(self):
        while True:
            # print out current location
            self.get_logger().info(f"Current pose: [{self.x},{self.y},{self.yaw}]")
            input = [self.desired_x, self.desired_y, self.desired_yaw]
            result = compute_motor_velocities(input, self.robot)
            self.get_logger().info(f"Computed wheel velocities: {result}")
            if (self.serial_connected):
                try:
                    msg = "["
                    for i in range(len(result)):
                        msg = msg + str(int(result[i]))
                        if (i != (len(result) - 1)):
                            msg = msg + ","
                    msg = msg + "]"
                    encoded_data = msg.encode()
                    self.get_logger().info(f"Sending encoded data: {encoded_data}")
                    self.ser.write(encoded_data)

                    # publish odometry
                    dt = 0.025
                    dx = (self.vx * np.cos(self.yaw) - self.vy * np.sin(self.yaw)) * dt
                    dy = (self.vx * np.sin(self.yaw) + self.vy * np.cos(self.yaw)) * dt
                    # Here we add some tolerance to the rotation
                    dth = self.vyaw * dt
                    self.x = self.x + dx
                    self.y = self.y + dy
                    self.yaw = self.yaw + dth
                    quat = self.euler_to_quaternion(0, 0, self.yaw)
                    current_time = self.get_clock().now().to_msg()

                    t = TransformStamped()
                    t.header.stamp = current_time
                    t.header.frame_id = "odom"
                    t.child_frame_id = "base_link"
                    t.transform.translation.x = self.x
                    t.transform.translation.y = self.y
                    t.transform.translation.z = 0.0
                    t.transform.rotation.x = quat[0]
                    t.transform.rotation.y = quat[1]
                    t.transform.rotation.z = quat[2]
                    t.transform.rotation.w = quat[3]
                    self.odom_broadcaster.sendTransform(t)

                    odom = Odometry()
                    odom.header.frame_id = "odom"
                    odom.header.stamp = current_time
                    # set the position
                    odom.pose.pose.position.x = self.x
                    odom.pose.pose.position.y = self.y
                    odom.pose.pose.position.z = 0.0
                    odom.pose.pose.orientation.x = quat[0]
                    odom.pose.pose.orientation.y = quat[1]
                    odom.pose.pose.orientation.z = quat[2]
                    odom.pose.pose.orientation.w = quat[3]
                    # set the velocity
                    odom.child_frame_id = "base_link"
                    odom.twist.twist.linear.x = self.vx * self.robot.max_v
                    odom.twist.twist.linear.y = self.vy * self.robot.max_v
                    odom.twist.twist.angular.z = self.vyaw * self.robot.max_w
                    self.odom_pub.publish(odom)

                except:
                    self.get_logger().error(f"Unable to write wheel velocities to serial port")
                    self.serial_connected = False
            sleep(0.025)


def main(argv=sys.argv):
    rclpy.init(args=argv)

    n = BaseController()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

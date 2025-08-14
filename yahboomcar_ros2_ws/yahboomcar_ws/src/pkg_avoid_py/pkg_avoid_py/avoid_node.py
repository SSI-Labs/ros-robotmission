import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

RAD2DEG = 180 / math.pi

class ObstacleMaze(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub_laser = self.create_subscription(LaserScan, "/scan", self.register_scan, 1)
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 1)

        self.declare_parameter("ResponseDist", 0.2)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("linear", 0.15)
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular", 1.0)
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle", 45.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value

        self.timer = self.create_timer(0.01, self.on_timer)

    def on_timer(self):
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value

    def register_scan(self, scan_data):
        if not isinstance(scan_data, LaserScan):
            return

        ranges = np.array(scan_data.ranges)
        Right_warning = 0
        Left_warning = 0
        front_warning = 0

        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if angle > 180:
                angle -= 360
            if 20 < angle < self.LaserAngle:
                if ranges[i] < self.ResponseDist * 1.5:
                    Left_warning += 1
            if -self.LaserAngle < angle < -20:
                if ranges[i] < self.ResponseDist * 1.5:
                    Right_warning += 1
            if abs(angle) <= 20:
                if ranges[i] <= self.ResponseDist * 1.5:
                    front_warning += 1

        twist = Twist()
        if front_warning > 10:
            if Left_warning > 10 and Right_warning > 10:
                twist.angular.z = -self.angular
            elif Right_warning > 10:
                twist.angular.z = self.angular
            else:
                twist.angular.z = -self.angular
        elif Left_warning > 10:
            twist.angular.z = -self.angular
        elif Right_warning > 10:
            twist.angular.z = self.angular
        else:
            twist.linear.x = self.linear

        self.pub_vel.publish(twist)

def main():
    rclpy.init()
    obstacle_maze = ObstacleMaze("obstacle_maze")
    print("3...2...1...Begin!")
    try:
        rclpy.spin(obstacle_maze)
    except KeyboardInterrupt:
        obstacle_maze.pub_vel.publish(Twist())
        obstacle_maze.destroy_node()
        rclpy.shutdown()
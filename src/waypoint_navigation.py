#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoint_1_x', 0.0), ('waypoint_1_y', 0.0),
                ('waypoint_2_x', 0.0), ('waypoint_2_y', 0.0),
                ('kp', 1.0), ('ki', 0.0), ('kd', 0.0)
            ]
        )
        
        self.waypoints = [
            (self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
            (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)
        ]
        self.current_waypoint_index = 0
        
        self.pid_linear = PIDController(
            self.get_parameter('kp').value, 
            self.get_parameter('ki').value, 
            self.get_parameter('kd').value
        )
        self.pid_angular = PIDController(2.0 * self.get_parameter('kp').value, 0.0, 0.0)

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update_navigation)
        
        self.position = [0.0, 0.0]
        self.yaw = 0.0
        self.reached_goal = False
        self.obstacle_detected = False
        self.safe_distance = 0.9
        self.get_logger().info("Robot waypoint navigation started.")


    def odom_callback(self, msg):
        self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance < self.safe_distance


    def update_navigation(self):
        if self.reached_goal:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_robot()
            self.reached_goal = True
            return
        
        if self.obstacle_detected:
            self.avoid_obstacle()
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        distance = math.sqrt((target_x - self.position[0])**2 + (target_y - self.position[1])**2)
        angle_to_target = math.atan2(target_y - self.position[1], target_x - self.position[0])
        angle_error = angle_to_target - self.yaw
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        
        dt = 0.1
        linear_speed = self.pid_linear.compute(0.0, -distance, dt)
        angular_speed = self.pid_angular.compute(0.0, -angle_error, dt)
        
        if distance < 0.1:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached: ({target_x}, {target_y})")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.stop_robot()
                self.reached_goal = True
                return
        
        twist_msg = Twist()
        twist_msg.linear.x = max(min(linear_speed, 0.2), -0.2)
        twist_msg.angular.z = max(min(angular_speed, 1.0), -1.0)
        self.vel_publisher.publish(twist_msg)

    def avoid_obstacle(self):
        self.get_logger().info("Obstacle detected! Stopping.")
        self.stop_robot()
        time.sleep(1)
        
        twist_msg = Twist()
        twist_msg.angular.z = 0.5  # Rotate left to avoid
        self.vel_publisher.publish(twist_msg)
        self.get_logger().info("Turning left to avoid obstacle.")
        
        time.sleep(2)  # Rotate for 2 seconds
        
        twist_msg.angular.z = 0.0
        twist_msg.linear.x = 0.2  # Move forward to bypass obstacle
        self.vel_publisher.publish(twist_msg)
        
        time.sleep(3)  # Move past obstacle
        
        self.get_logger().info("Resuming waypoint navigation.")

    def stop_robot(self):
        stop_msg = Twist()
        self.vel_publisher.publish(stop_msg)
        


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

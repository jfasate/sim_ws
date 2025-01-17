#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from time import gmtime, strftime
import os
#import tf
from os.path import expanduser
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')
        
        # Save as test.csv in home directory
        filename = os.path.expanduser('~/test.csv')
        self.file = open(filename, 'w')
        self.get_logger().info(f'Created file: {filename}')
        
        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.save_waypoint,
            10)
        
        self.waypoint_count = 0
        self.get_logger().info('Saving waypoints...')
    
    def save_waypoint(self, data):
        try:
            # Extract quaternion
            quaternion = np.array([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w
            ])
            
            # Convert to euler angles
            euler = euler_from_quaternion(quaternion)
            
            # Calculate speed
            speed = LA.norm(np.array([
                data.twist.twist.linear.x,
                data.twist.twist.linear.y,
                data.twist.twist.linear.z
            ]), 2)
            
            if data.twist.twist.linear.x > 0.:
                self.get_logger().info(f'Forward velocity: {data.twist.twist.linear.x}')
            
            self.file.write(f'{data.pose.pose.position.x}, {data.pose.pose.position.y}, {euler[2]}, {speed}\n')
                
        except Exception as e:
            self.get_logger().error(f'Error saving waypoint: {str(e)}')
    
    def shutdown(self):
        self.file.flush()
        self.file.close()
        self.get_logger().info(f'Recorded total of {self.waypoint_count} waypoints')
        self.get_logger().info('Goodbye')

def main():
    rclpy.init()
    waypoints_logger = WaypointsLogger()
    
    try:
        rclpy.spin(waypoints_logger)
    except KeyboardInterrupt:
        #waypoints_logger.shutdown()
        pass
    finally:
        #waypoints_logger.shutdown()
        waypoints_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
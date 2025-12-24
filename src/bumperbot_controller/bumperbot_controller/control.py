#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped 
from aruco_interfaces.msg import ArucoMarkers
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import math
import tf2_geometry_msgs 

class SimpleArucoFollower(Node):
    def __init__(self):
        super().__init__('simple_aruco_follower')

        self.target_id = 35
        self.stop_distance = 0.8 
        self.k_linear = 0.3    
        self.k_angular = 1.0   
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.5
        
        self.search_timeout = 2.0
        self.search_speed = 0.4
        
        self.target_frame = "base_footprint" 
        
        self.pub_cmd = self.create_publisher(
            TwistStamped, 
            '/bumperbot_controller/cmd_vel', 
            10)
        
        self.sub_aruco = self.create_subscription(
            ArucoMarkers,
            '/aruco/markers',
            self.listener_callback,
            10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_seen_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_callback)
        
        self.get_logger().info("Follower Started: TwistStamped Mode + Auto Search")

    def listener_callback(self, msg):
        target_pose = None
        for i, m_id in enumerate(msg.marker_ids):
            if m_id == self.target_id:
                target_pose = msg.poses[i]
                break
        
        if target_pose:
            self.last_seen_time = self.get_clock().now()
            self.drive_robot(msg.header, target_pose)

    def drive_robot(self, header_msg, pose_msg):
        try:
            source_pose = PoseStamped()
            source_pose.header = header_msg
            source_pose.pose = pose_msg
            transform = self.tf_buffer.transform(source_pose, self.target_frame, timeout=Duration(seconds=0.05))
            x = transform.pose.position.x
            y = transform.pose.position.y
            distance_error = x - self.stop_distance
            angle_error = math.atan2(y, x) 
            
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = self.target_frame
            
            angular_z = angle_error * self.k_angular
            linear_x = 0.0

            if abs(angle_error) < 0.2: 
                linear_x = distance_error * self.k_linear
            else:
                linear_x = 0.0 

            linear_x = min(self.max_linear_speed, max(-self.max_linear_speed, linear_x))
            angular_z = min(self.max_angular_speed, max(-self.max_angular_speed, angular_z))
            
            if abs(distance_error) < 0.05:
                linear_x = 0.0

            cmd.twist.linear.x = float(linear_x)
            cmd.twist.angular.z = float(angular_z)

            self.pub_cmd.publish(cmd)

        except Exception as e:
            self.get_logger().warn(f"TF Error: {e}")

    def watchdog_callback(self):
        time_since_seen = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.target_frame
        
        if time_since_seen > self.search_timeout:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = self.search_speed
            self.pub_cmd.publish(cmd)
            self.get_logger().info("Searching...", throttle_duration_sec=2.0)
            
        elif time_since_seen > 0.5:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleArucoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
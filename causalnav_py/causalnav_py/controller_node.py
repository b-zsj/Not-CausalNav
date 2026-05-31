import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

class APFController(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.current_route = []
        self.active_goal = None
        self.current_pose = None
        self.dynamic_obstacles = []
        
        # --- Tuning Parameters ---
        self.k_attract = 1.0           
        self.k_repel = 2.5             
        self.repel_radius = 2.0        # How close an obstacle must be to trigger evasion
        self.waypoint_tolerance = 1.0  # Distance to consider a waypoint "reached"
        
        # Jetbot Kinematic Limits
        self.max_linear_speed = 0.6    
        self.max_angular_speed = 1.2   
        
        self.route_sub = self.create_subscription(PoseArray, '/planned_route', self.route_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_cb, 10)
        self.obs_sub = self.create_subscription(PoseArray, '/causalnav/dynamic_obstacles', self.obs_cb, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop) 
        
        self.get_logger().info("APF Controller Online.")

    def route_cb(self, msg):
        self.current_route = msg.poses
        self.dispatch_next_waypoint()

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def obs_cb(self, msg):
        self.dynamic_obstacles = msg.poses

    def dispatch_next_waypoint(self):
        if not self.current_route:
            self.active_goal = None
            return
        self.active_goal = self.current_route.pop(0)
        self.get_logger().info(f"Driving to waypoint: ({self.active_goal.position.x:.2f}, {self.active_goal.position.y:.2f})")

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.active_goal is None or self.current_pose is None:
            self.stop_robot()
            return

        rx, ry = self.current_pose.position.x, self.current_pose.position.y
        gx, gy = self.active_goal.position.x, self.active_goal.position.y

        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal < self.waypoint_tolerance:
            self.dispatch_next_waypoint()
            return

        # 1. Attractor Vector
        f_attract_x = self.k_attract * (gx - rx)
        f_attract_y = self.k_attract * (gy - ry)

        # 2. Repulsor Vector
        f_repel_x, f_repel_y = 0.0, 0.0
        for obs in self.dynamic_obstacles:
            ox, oy = obs.position.x, obs.position.y
            dist_to_obs = math.hypot(rx - ox, ry - oy)
            
            if dist_to_obs < self.repel_radius and dist_to_obs > 0.01:
                force_mag = self.k_repel * ((1.0 / dist_to_obs) - (1.0 / self.repel_radius)) * (1.0 / (dist_to_obs**2))
                f_repel_x += force_mag * ((rx - ox) / dist_to_obs)
                f_repel_y += force_mag * ((ry - oy) / dist_to_obs)

        result_x = f_attract_x + f_repel_x
        result_y = f_attract_y + f_repel_y

        target_heading = math.atan2(result_y, result_x)
        current_heading = self.get_yaw(self.current_pose.orientation)
        
        heading_error = math.atan2(math.sin(target_heading - current_heading), math.cos(target_heading - current_heading))

        cmd = Twist()
        cmd.angular.z = np.clip(2.0 * heading_error, -self.max_angular_speed, self.max_angular_speed)

        if abs(heading_error) > 0.8:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = np.clip(math.hypot(result_x, result_y), 0.0, self.max_linear_speed)
            
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = APFController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
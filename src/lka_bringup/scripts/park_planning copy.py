#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from tf_transformations import euler_from_quaternion

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')
        
        # Vehicle Dimensions
        self.a = 3.005
        self.delta_max = math.radians(35.0)
        self.rear_axle_offset = 1.42    # from origin backward
        self.b = 1.67 / 2.0             # Half track
        self.d_front = 0.81             # Front overhang
        self.d_rear = 0.98              # Rear overhang
        self.d_side = 0.25              # Side distance to mirror
        self.L = 6.2                    # Parking spot length 
        
        self.center_start_pose = None                        # initial pose (car's center) (x, y, yaw)
        self.center_target_pose = (69.9, -105.0, 0.0)        # target pose (car's center) (x, y, yaw)
        self.init_pose_captured = False                     

        self.cmd_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        self.odom_sub = self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.odom_callback, 10)

        self.plan = None
        self.current_state = 'EVALUATING'
        self.is_multiple_trials = False
        self.trials_completed = 1
        self.total_trials = 1
        self.shunt_displacement = 0.0
        self.target_yaw = 0.0


    def get_rear_axle(self, x, y, yaw):
        # convert center car representation to match paper's rear axle 
        ex = x - self.rear_axle_offset * math.cos(yaw)
        ey = y - self.rear_axle_offset * math.sin(yaw)
        return (ex, ey, yaw)

    def calculate_path(self, start, target, silent=False):
        x_s, y_s, yaw_s = self.get_rear_axle(*start)
        x_t, y_t, yaw_t = self.get_rear_axle(*target)

        dx_g = x_t - x_s
        dy_g = y_t - y_s
        is_left_parking = (math.cos(yaw_s) * dy_g - math.sin(yaw_s) * dx_g) > 0
        
        RE_min = self.a / math.tan(self.delta_max)

        # Minimum parking spot
        R_Bl_min = math.sqrt((RE_min + self.b + self.d_side)**2 + (self.a + self.d_front)**2)
        L_min = self.d_rear + math.sqrt(R_Bl_min**2 - (RE_min - self.b - self.d_side)**2)
        
        if not silent:
            self.get_logger().info(f"Lmin: {L_min:.4f} m")

        # N trials
        if self.L < L_min and self.L > (self.a + self.d_front + self.d_rear):
            if not self.is_multiple_trials:
                self.is_multiple_trials = True
                if not silent:
                    self.get_logger().info("L < L_min, implement n trials")
                l = self.L - (self.a + self.d_front + self.d_rear)
                
                # calculate exact 'd': d = y_F - y_F1
                x_F = self.L - self.d_rear
                if R_Bl_min > x_F:
                    y_F1 = RE_min - math.sqrt(R_Bl_min**2 - x_F**2)
                else:
                    y_F1 = RE_min
                
                y_F = self.b + self.d_side
                d_offset = abs(y_F - y_F1)
                
                # Using eqn.7 to find lateral displacement per shunt move
                delta_y = 2 * (RE_min - math.sqrt(max(0, RE_min**2 - (l/2.0)**2)))
                
                # Total trials using eqn.8 
                self.total_trials = int(d_offset / delta_y) + 2
                
                # Update the target park position to the nearest parallel position
                offset_y = -d_offset if is_left_parking else d_offset
                self.center_target_pose = (target[0], target[1] + offset_y, target[2])
                
                if not silent:
                    self.get_logger().info(f"New target: {self.center_target_pose[0]:.2f}, {self.center_target_pose[1]:.2f}, {math.degrees(self.center_target_pose[2]):.2f} deg")
                    self.get_logger().info(f"Number of trials: {self.total_trials}. Lat offset: {d_offset:.2f}")
                
                # Re-fetch new rear axle target using the new updated center_target_pose
                x_t, y_t, yaw_t = self.get_rear_axle(*self.center_target_pose)

        if is_left_parking:
            C_t_x = x_t + RE_min * math.cos(yaw_t - math.pi / 2)
            C_t_y = y_t + RE_min * math.sin(yaw_t - math.pi / 2)
            
            dx = C_t_x - x_s
            dy = C_t_y - y_s
            d_C_Einit = math.sqrt(dx**2 + dy**2)

            dir_x = math.cos(yaw_s + math.pi / 2)
            dir_y = math.sin(yaw_s + math.pi / 2)
        else:
            C_t_x = x_t + RE_min * math.cos(yaw_t + math.pi / 2)
            C_t_y = y_t + RE_min * math.sin(yaw_t + math.pi / 2)
            
            dx = C_t_x - x_s
            dy = C_t_y - y_s
            d_C_Einit = math.sqrt(dx**2 + dy**2)
            
            dir_x = math.cos(yaw_s - math.pi / 2)
            dir_y = math.sin(yaw_s - math.pi / 2)

        cos_alpha = (dx * dir_x + dy * dir_y) / d_C_Einit
        
        # Admissible Circle / Feasibility 
        d_EC_lmin = RE_min * cos_alpha + math.sqrt((RE_min * cos_alpha)**2 + RE_min**2 + 2 * RE_min**2)
        if d_C_Einit < 1.05*d_EC_lmin:
            if not silent:
                self.get_logger().warn(f"d_C_Einit: {d_C_Einit:.2f} < d_EC_lmin: {1.05*d_EC_lmin:.2f}, Move forward")
            return None

        num = d_C_Einit**2 - RE_min**2
        den = 2 * RE_min + 2 * d_C_Einit * cos_alpha
        
        if den == 0: return None
        RE_init = num / den

        if RE_init < RE_min:
            if not silent:
                self.get_logger().warn(f"RE_init: {RE_init} < RE_min: {RE_min}")
            return None

        # Center of parking first arc
        C_i_x = x_s + RE_init * dir_x
        C_i_y = y_s + RE_init * dir_y
        
        # Tangency point T_e (between C_i and C_t)
        d_Ci_Ct = RE_init + RE_min
        T_e_x = C_i_x + (RE_init / d_Ci_Ct) * (C_t_x - C_i_x)
        T_e_y = C_i_y + (RE_init / d_Ci_Ct) * (C_t_y - C_i_y)

        delta_init = math.atan(self.a / RE_init)
        
        if is_left_parking:
            first_steer = -delta_init
            second_steer = self.delta_max 
        else:
            first_steer = delta_init
            second_steer = -self.delta_max
            
        return {
            'T_e': (T_e_x, T_e_y),
            'first_steer': first_steer,
            'second_steer': second_steer,
            'is_left': is_left_parking,
            'C_i': (C_i_x, C_i_y)
        }

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        x_c = msg.pose.pose.position.x
        y_c = msg.pose.pose.position.y
        _, _, yaw_e = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self.init_pose_captured:
            self.center_start_pose = (x_c, y_c, yaw_e)
            self.init_pose_captured = True
            self.get_logger().info(
                f"Initial pose: "
                f"X={x_c:.2f}, Y={y_c:.2f}, Yaw={math.degrees(yaw_e):.2f} deg"
            )

        x_e, y_e, yaw_e = self.get_rear_axle(x_c, y_c, yaw_e)
        
        control = CarlaEgoVehicleControl()
        control.reverse = True
        
        if self.current_state == 'EVALUATING':
            self.plan = self.calculate_path((x_c, y_c, yaw_e), self.center_target_pose, silent=False)
            if self.plan is None:
                self.current_state = 'FORWARD_ADJUST'
            else:
                self.current_state = 'WAIT_STEERING'
                self.start_time = self.get_clock().now()
                
        elif self.current_state == 'FORWARD_ADJUST':
            control.steer = 0.0
            control.throttle = 0.3
            control.reverse = False
            
            # Check if feasible path exists while moving forward
            plan = self.calculate_path((x_c, y_c, yaw_e), self.center_target_pose, silent=True)
            if plan is not None:
                control.throttle = 0.0
                control.brake = 1.0

                self.current_state = 'WAIT_FOR_STOP'
                self.get_logger().info('Feasible path found.')

        elif self.current_state == 'WAIT_FOR_STOP':
            control.throttle = 0.0
            control.brake = 1.0
            linear_velocity = msg.twist.twist.linear
            speed = math.hypot(linear_velocity.x, linear_velocity.y)

            if speed < 0.01:
                self.center_start_pose = (x_c, y_c, yaw_e) 
                
                self.plan = self.calculate_path(self.center_start_pose, self.center_target_pose, silent=False)
                self.current_state = 'WAIT_STEERING'
                self.start_time = self.get_clock().now()
                self.get_logger().info(f"Initial pose using for planning: X={x_c:.2f}, Y={y_c:.2f}")

        elif self.current_state == 'WAIT_STEERING':
            if not self.plan:
                return
            control.steer = self.plan['first_steer']
            control.throttle = 0.0
            control.brake = 1.0
            
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > 1.5:
                self.current_state = 'FIRST_ARC'
                self.get_logger().info('Begin FIRST_ARC')
                
        elif self.current_state == 'FIRST_ARC':
            if not self.plan:
                return
            control.steer = self.plan['first_steer']
            control.throttle = 0.3
            
            # Check if reached tangency point (T_e)
            T_e_x, T_e_y = self.plan['T_e']
            dist = math.sqrt((x_e - T_e_x)**2 + (y_e - T_e_y)**2)

            # Track minimum distance to T_e to detect when start moving away
            if not hasattr(self, 'min_dist'):
                self.min_dist = dist
            if dist < self.min_dist:
                self.min_dist = dist
            
            # Transition if very close to T_e
            if dist < 0.3 or (dist < 0.3 and dist > self.min_dist + 0.1):
                self.current_state = 'WAIT_STEERING_2'
                self.start_time = self.get_clock().now()
                self.get_logger().info('Transitioning to WAIT_STEERING_2')

        elif self.current_state == 'WAIT_STEERING_2':
            if not self.plan:
                return
            control.steer = self.plan['second_steer']
            control.throttle = 0.0
            control.brake = 1.0
            
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > 1.5:
                self.current_state = 'SECOND_ARC'
                self.get_logger().info('Begin SECOND_ARC')
                
        elif self.current_state == 'SECOND_ARC':
            if not self.plan:
                return
            control.steer = self.plan['second_steer']
            control.throttle = 0.3
            
            # Single trial stop condition 
            yaw_diff = abs(yaw_e - self.center_target_pose[2])
            if yaw_diff < math.radians(1.5) and math.sqrt((x_e - self.center_target_pose[0])**2 + (y_e - self.center_target_pose[1])**2) < 0.1:
                control.steer = 0.0
                control.throttle = 0.0
                control.brake = 1.0
                
                if self.is_multiple_trials and self.trials_completed < self.total_trials:
                    l = self.L - (self.a + self.d_front + self.d_rear)
                    self.limit_front = self.center_target_pose[0] + (l / 2.05)
                    self.limit_rear  = self.center_target_pose[0] - (l / 2.05)
                    
                    if x_c < self.center_target_pose[0]:
                        self.current_state = 'SHUNT_BACKWARD_1'
                        self.shunt_mid_x = x_c + (self.limit_rear - x_c) * 0.4 
                    else:
                        self.current_state = 'SHUNT_FORWARD_1'
                        self.shunt_mid_x = x_c + (self.limit_front - x_c) * 0.4      

                    self.target_yaw = self.center_target_pose[2]
                    self.trials_completed += 1
                    
                    self.get_logger().info(f"Single trial complete, Pose: X={x_c:.2f}, Y={y_c:.2f}, Yaw={math.degrees(yaw_e):.2f} deg")
                    self.get_logger().info(f"Starting multiple parallel trials... [{self.trials_completed}/{self.total_trials}]")
                    self.get_logger().info(f"Limits: Front={self.limit_front:.2f}, Rear={self.limit_rear:.2f}")
                else:
                    self.current_state = 'DONE'
                    self.get_logger().info('Parking complete.')

        # Multiple Trial Shunting Logic 
        elif self.current_state == 'SHUNT_FORWARD_1':
            control.reverse = False
            control.throttle = 0.2
            control.brake = 0.0
            
            control.steer = -self.delta_max if self.plan['is_left'] else self.delta_max
            
            if x_c >= self.shunt_mid_x:
                self.current_state = 'SHUNT_FORWARD_2'
                self.get_logger().info('Switching to SHUNT_FORWARD_2')

        elif self.current_state == 'SHUNT_FORWARD_2':
            control.reverse = False
            control.throttle = 0.2
            control.brake = 0.0
            control.steer = self.delta_max if self.plan['is_left'] else -self.delta_max
            
            yaw_diff = abs(yaw_e - self.target_yaw)
            # Terminate if hit X bound, OR if yaw is corrected AND we are safely past the midpoint
            if x_c >= self.limit_front or (yaw_diff < math.radians(1.0) and x_c > self.shunt_mid_x + 0.05): 
                control.brake = 1.0
                control.throttle = 0.0
                control.steer = 0.0
                self.start_time = self.get_clock().now()
                self.current_state = 'WAIT_BEFORE_BACKWARD'
                self.get_logger().info(f'Switching to back shunt, X={x_c:.2f}, Yaw Error={math.degrees(yaw_diff):.2f} deg')

        elif self.current_state == 'WAIT_BEFORE_BACKWARD':
            control.brake = 1.0
            control.throttle = 0.0
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > 1.0:
                self.current_state = 'SHUNT_BACKWARD_1'
                self.shunt_mid_x = x_c + (self.limit_rear - x_c) * 0.4

        elif self.current_state == 'SHUNT_BACKWARD_1':
            control.reverse = True
            control.throttle = 0.2
            control.brake = 0.0
            
            control.steer = -self.delta_max if self.plan['is_left'] else self.delta_max
            
            if x_c <= self.shunt_mid_x:
                self.current_state = 'SHUNT_BACKWARD_2'
                self.get_logger().info('Switching to SHUNT_BACKWARD_2')

        elif self.current_state == 'SHUNT_BACKWARD_2':
            control.reverse = True
            control.throttle = 0.2
            control.brake = 0.0
            control.steer = self.delta_max if self.plan['is_left'] else -self.delta_max
            
            yaw_diff = abs(yaw_e - self.target_yaw)
            # Terminate if hit X bound, OR if yaw is corrected AND we are safely past the midpoint
            if x_c <= self.limit_rear or (yaw_diff < math.radians(1.0) and x_c < self.shunt_mid_x - 0.05):
                control.brake = 1.0
                control.throttle = 0.0
                control.steer = 0.0
                
                self.trials_completed += 1
                if self.trials_completed >= self.total_trials:
                    self.current_state = 'DONE'
                    self.get_logger().info('Multiple trials parking complete.')
                else:
                    self.current_state = 'WAIT_BEFORE_FORWARD'
                    self.start_time = self.get_clock().now()
                    self.get_logger().info(f"Shunt loop [{self.trials_completed}/{self.total_trials}] complete. Position: X={x_c:.2f}, Y={y_c:.2f}, Yaw={math.degrees(yaw_e):.2f} deg")
                    
        elif self.current_state == 'WAIT_BEFORE_FORWARD':
            control.brake = 1.0
            control.throttle = 0.0
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > 1.0:
                self.current_state = 'SHUNT_FORWARD_1'
                self.shunt_mid_x = x_c + (self.limit_front - x_c) * 0.4

        elif self.current_state == 'DONE':
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            
        self.cmd_pub.publish(control)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
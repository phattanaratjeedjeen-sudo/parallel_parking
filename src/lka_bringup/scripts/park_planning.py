#!/usr/bin/env python3
import json
import math
import os
import datetime
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

class LoggerWrapper:
    def __init__(self, ros_logger, file_handle):
        self.ros_logger = ros_logger
        self.file_handle = file_handle
        
    def _write(self, level, msg):
        if self.file_handle and not self.file_handle.closed:
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self.file_handle.write(f"[{timestamp}] [{level}] {msg}\n")
            self.file_handle.flush()
            
    def info(self, msg):
        self.ros_logger.info(msg)
        self._write('INFO', msg)
        
    def warn(self, msg):
        if hasattr(self.ros_logger, 'warning'):
            self.ros_logger.warning(msg)
        else:
            self.ros_logger.warn(msg)
        self._write('WARN', msg)
        
    def error(self, msg):
        self.ros_logger.error(msg)
        self._write('ERROR', msg)
        
    def fatal(self, msg):
        self.ros_logger.fatal(msg)
        self._write('FATAL', msg)
        
    def debug(self, msg):
        self.ros_logger.debug(msg)
        self._write('DEBUG', msg)

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')
        
        # vehicle parameters
        self.declare_parameter('max_steer', 35.0)
        self.delta_max = math.radians(self.get_parameter('max_steer').value)
        self.a = 3.005                  # from rear axle to front axle     
        self.rear_axle_offset = 1.42    # from origin backward
        self.b = 1.67 / 2.0             # half track
        self.d_front = 0.81             # front overhang
        self.d_rear = 0.98              # rear overhang
        self.d_side = 0.25              # side distance to mirror
        self.declare_parameter('L', 6.7)
        self.L = self.get_parameter('L').value
        
        self.center_target_pose = (69.9, -105.0, 0.0)   # target pose (car's center) (x, y, yaw)
        self.center_start_pose = None                   # initial pose (car's center) (x, y, yaw) 
        self.offset_limit = 0.10                        # limit offset for shunt adjustments (m)  
        self.init_pose_captured = False    
        self.plan = None
        self.is_multiple_trials = False
        self.current_state = 'EVALUATING'
        self.trials_completed = 0                       # for multi trials
        self.total_trials = 1                           # for multi trials
        self.change_gear_times = 0                      
        self.plot_triggered = False

        self.log_dir = os.path.expanduser('~/park_ws/src/lka_bringup/data/results/logger')
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = self.get_next_log_filename()
        self.log_file_handle = open(self.log_file, 'w', encoding='utf-8')
        self._custom_logger = LoggerWrapper(super().get_logger(), self.log_file_handle)

        self.cmd_pub = self.create_publisher(
            CarlaEgoVehicleControl, 
            '/carla/ego_vehicle/vehicle_control_cmd', 
            10
        )
        self.gear_pub = self.create_publisher(Int32, 
            '/carla/ego_vehicle/gear_changes', 
            10
        )
        self.config_srv = self.create_service(
            Trigger, 
            '/carla/ego_vehicle/get_park_config', 
            self.get_config_callback
        )
        self.plot_client = self.create_client(
            Trigger,
            '/carla/ego_vehicle/plot_results'
        )
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/carla/ego_vehicle/odometry', 
            self.odom_callback, 
            10
        )

    def get_logger(self):
        if not hasattr(self, '_custom_logger'):
            return super().get_logger()
        return self._custom_logger

    def get_next_log_filename(self):
        index = 1
        max_steer_deg = round(math.degrees(self.delta_max), 1)
        while True:
            filename = os.path.join(self.log_dir, f'park_{self.L}_{max_steer_deg}_{index}.log')
            if not os.path.exists(filename):
                return filename
            index += 1
        
    def get_config_callback(self, request, response):
        config_data = {
            "L": self.L,
            "max_steer": round(math.degrees(self.delta_max), 1),
            "car_length": self.a + self.d_front + self.d_rear
        }
        response.success = True
        response.message = json.dumps(config_data)
        return response
        
    def trigger_plot(self):
        if self.plot_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.plot_client.call_async(req)
            future.add_done_callback(self.plot_response_callback)
        else:
            self.get_logger().error('Plot service not available')
            
    def plot_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Plot response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Failed to call plot service: {e}")

    def get_rear_axle(self, x, y, yaw):
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

        # minimum parking spot
        R_Bt_min = math.sqrt((RE_min + self.b + self.d_side)**2 + (self.a + self.d_front)**2)
        L_min = self.d_rear + math.sqrt(R_Bt_min**2 - (RE_min - self.b - self.d_side)**2)
        
        if not silent:
            self.get_logger().info(f"Lmin: {L_min:.4f} m")

        # N trials
        if self.L < L_min and self.L > (self.a + self.d_front + self.d_rear):
            if not self.is_multiple_trials:
                self.is_multiple_trials = True
                if not silent:
                    self.get_logger().info("L < L_min, implement n trials")
                l = self.L - (self.a + self.d_front + self.d_rear)
                
                x_F = self.L - self.d_rear
                if R_Bt_min > x_F:
                    y_F1 = RE_min - math.sqrt(R_Bt_min**2 - x_F**2)
                else:
                    y_F1 = RE_min
                
                y_F = self.b + self.d_side
                d_offset = abs(y_F - y_F1)
                
                # lateral displacement per shunt move (adjusted for safety offset limits)
                l_stroke = max(0.01, l - 2 * self.offset_limit)
                delta_y = 2 * (RE_min - math.sqrt(max(0, RE_min**2 - (l_stroke/2.0)**2)))
                
                # total loops needed (each loop is 1 forward + 1 backward = 2 strokes)
                # lateral displacement per loop is 2.0 * delta_y
                self.total_trials = math.floor(d_offset / (2.0 * delta_y) + 0.5)
                
                # update the target park position to the nearest parallel position
                offset_y = -d_offset if is_left_parking else d_offset
                self.center_target_pose = (target[0], target[1] + offset_y, target[2])
                
                if not silent:
                    self.get_logger().info(f"New target: {self.center_target_pose[0]:.2f}, {self.center_target_pose[1]:.2f}, {math.degrees(self.center_target_pose[2]):.2f} deg")
                    self.get_logger().info(f"Number of trials: {self.total_trials}. Lat offset: {d_offset:.2f}")
                
                # re-fetch new rear axle target using the new updated center_target_pose
                x_t, y_t, yaw_t = self.get_rear_axle(*self.center_target_pose)

        if is_left_parking:
            C_t_x = x_t + RE_min * math.cos(yaw_t - math.pi / 2)
            C_t_y = y_t + RE_min * math.sin(yaw_t - math.pi / 2)
            dx = C_t_x - x_s
            dy = C_t_y - y_s
            d_Ct_Einit = math.sqrt(dx**2 + dy**2)
            dir_x = math.cos(yaw_s + math.pi / 2)
            dir_y = math.sin(yaw_s + math.pi / 2)
        
        else:
            C_t_x = x_t + RE_min * math.cos(yaw_t + math.pi / 2)
            C_t_y = y_t + RE_min * math.sin(yaw_t + math.pi / 2)
            dx = C_t_x - x_s
            dy = C_t_y - y_s
            d_Ct_Einit = math.sqrt(dx**2 + dy**2)
            dir_x = math.cos(yaw_s - math.pi / 2)
            dir_y = math.sin(yaw_s - math.pi / 2)

        cos_alpha = (dx * dir_x + dy * dir_y) / d_Ct_Einit
        
        # feasibility 
        d_Ct_Einit_min = RE_min * cos_alpha + math.sqrt((RE_min * cos_alpha)**2 + RE_min**2 + 2 * RE_min**2)
        if d_Ct_Einit < 1.05*d_Ct_Einit_min:
            if not silent:
                self.get_logger().warn(f"d_Ct_Einit: {d_Ct_Einit:.2f} < d_Ct_Einit_min: {1.05*d_Ct_Einit_min:.2f}, Move forward")
            return None

        num = d_Ct_Einit**2 - RE_min**2
        den = 2 * RE_min + 2 * d_Ct_Einit * cos_alpha
        if den == 0: return None
        RE_init = num / den

        if RE_init < RE_min:
            if not silent:
                self.get_logger().warn(f"RE_init: {RE_init} < RE_min: {RE_min}")
            return None

        # center of parking first arc
        C_i_x = x_s + RE_init * dir_x
        C_i_y = y_s + RE_init * dir_y
        
        # tangency point T_e (between C_i and C_t)
        d_Ci_Ct = RE_init + RE_min
        T_e_x = C_i_x + (RE_init / d_Ci_Ct) * (C_t_x - C_i_x)
        T_e_y = C_i_y + (RE_init / d_Ci_Ct) * (C_t_y - C_i_y)

        # first steer
        delta_init = math.atan(self.a / RE_init)
        
        if is_left_parking:
            first_steer = -delta_init
            second_steer = self.delta_max 
        else:
            first_steer = delta_init
            second_steer = -self.delta_max
            
        return {
            'T_e': (T_e_x, T_e_y),          # tangent point between the two arcs
            'first_steer': first_steer,     # initial steering for first arc
            'second_steer': second_steer,   # steering for second arc
            'is_left': is_left_parking,     # parking side
            'C_i': (C_i_x, C_i_y)           # center of the first arc
        }


    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        x_c = msg.pose.pose.position.x
        y_c = msg.pose.pose.position.y
        _, _, yaw_e = euler_from_quaternion([q.x, q.y, q.z, q.w])
        linear_vel = msg.twist.twist.linear
        speed = math.hypot(linear_vel.x, linear_vel.y)

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
                self.change_gear_times += 1
            else:
                self.current_state = 'WAIT_STEERING'
                
        elif self.current_state == 'FORWARD_ADJUST':
            control.steer = 0.0
            control.throttle = 0.3
            control.reverse = False
            
            # check if feasible path exists while moving forward
            plan = self.calculate_path((x_c, y_c, yaw_e), self.center_target_pose, silent=True)
            if plan is not None:
                control.throttle = 0.0
                control.brake = 1.0
                self.current_state = 'WAIT_FOR_STOP'

        elif self.current_state == 'WAIT_FOR_STOP':
            control.throttle = 0.0
            control.brake = 1.0

            if speed < 0.001:
                self.center_start_pose = (x_c, y_c, yaw_e) 
                self.plan = self.calculate_path(self.center_start_pose, self.center_target_pose, silent=False)
                self.current_state = 'WAIT_STEERING'
                self.get_logger().info(f"Initial pose for planning: X={x_c:.2f}, Y={y_c:.2f}")

        elif self.current_state == 'WAIT_STEERING':
            if not self.plan: return
            control.steer = self.plan['first_steer']
            control.throttle = 0.0
            control.brake = 1.0
            
            if speed < 0.001:
                self.current_state = 'FIRST_ARC'
                self.change_gear_times += 1
                self.get_logger().info('Begin FIRST_ARC')
                
        elif self.current_state == 'FIRST_ARC':
            if not self.plan: return
            control.steer = self.plan['first_steer']
            control.throttle = 0.3
            
            # check if reached tangency point (T_e)
            T_e_x, T_e_y = self.plan['T_e']
            dist = math.sqrt((x_e - T_e_x)**2 + (y_e - T_e_y)**2)

            # track minimum distance to T_e to detect moving away
            if not hasattr(self, 'min_dist'):
                self.min_dist = dist
            if dist < self.min_dist:
                self.min_dist = dist
            
            # transition if close, or pass the closest point and moving away
            if dist < 0.25 or (self.min_dist < 0.4 and dist > self.min_dist + 0.05):
                self.current_state = 'WAIT_STEERING_2'
                self.get_logger().info('Transitioning to WAIT_STEERING_2')

        elif self.current_state == 'WAIT_STEERING_2':
            if not self.plan: return
            control.steer = self.plan['second_steer']
            control.throttle = 0.0
            control.brake = 1.0
            
            if speed < 0.001:
                self.current_state = 'SECOND_ARC'
                self.get_logger().info('Begin SECOND_ARC')
                
        elif self.current_state == 'SECOND_ARC':
            if not self.plan: return
            control.steer = self.plan['second_steer']
            control.throttle = 0.3
            
            # single trial stop condition 
            yaw_diff = abs(yaw_e - self.center_target_pose[2])
            dist = math.sqrt((x_c - self.center_target_pose[0])**2 + (y_c - self.center_target_pose[1])**2)
            if yaw_diff < math.radians(3) and dist < 0.25:
                self.get_logger().info(
                    "Single trial: complete, "
                    f"x_c: {x_c:.2f}, " 
                    f"y_c: {y_c:.2f}, "
                    f"yaw_e: {math.degrees(yaw_e):.2f} deg"
                )
                control.steer = 0.0
                control.throttle = 0.0
                control.brake = 1.0
                
                if self.is_multiple_trials and self.trials_completed < self.total_trials:
                    self.current_state = 'SHUNT_FORWARD_1'
                    self.change_gear_times += 1
                    l = self.L - (self.a + self.d_front + self.d_rear)                            # gap between car and obstacle when perfectly parked
                    self.limit_front = self.center_target_pose[0] + l - 2*self.offset_limit       # front limit (car's center X position)
                    self.limit_rear  = self.center_target_pose[0]                                 # rear limit (car's center X position)
                    self.shunt_mid_x = (x_c + self.limit_front) / 2.0                             # dynamic midpoint to ensure symmetric arcs
                    
                    self.get_logger().info(f"Start n trials (not include single section): {self.trials_completed + 1}/{self.total_trials}")
                    self.get_logger().info(f"X Limits: Front={self.limit_front:.2f}, Rear={self.limit_rear:.2f}")
                    self.get_logger().info(f"Shunt mid X: {self.shunt_mid_x:.2f}")
                    self.get_logger().info(f"Target position:{self.center_target_pose[0]:.2f}, {self.center_target_pose[1]:.2f}")
                else:
                    self.current_state = 'DONE'
                    self.get_logger().info('Single trial parking complete')

        # Shunt logic
        elif self.current_state == 'SHUNT_FORWARD_1':
            if not self.plan: return
            control.reverse = False
            control.throttle = 0.1
            control.brake = 0.0
            control.steer = -self.delta_max if self.plan['is_left'] else self.delta_max
            
            if x_c >= self.shunt_mid_x:
                self.current_state = 'SHUNT_FORWARD_2'
                self.get_logger().info('Switching to SHUNT_FORWARD_2')

        elif self.current_state == 'SHUNT_FORWARD_2':
            if not self.plan: return
            control.reverse = False
            control.throttle = 0.1
            control.brake = 0.0
            control.steer = self.delta_max if self.plan['is_left'] else -self.delta_max
            
            yaw_diff = abs(yaw_e - self.center_target_pose[2])
            if x_c >= self.limit_front or (yaw_diff < math.radians(0.1) and x_c >= self.shunt_mid_x + 0.05): 
                control.brake = 1.0
                control.throttle = 0.0
                control.steer = 0.0
                self.current_state = 'WAIT_BEFORE_BACKWARD'
                self.get_logger().info(f'Switching to back shunt, X={x_c:.2f}, Y={y_c:.2f}')

        elif self.current_state == 'WAIT_BEFORE_BACKWARD':
            control.brake = 1.0
            control.throttle = 0.0
            if speed < 0.001:
                self.current_state = 'SHUNT_BACKWARD_1'
                self.change_gear_times += 1
                self.shunt_mid_x = (x_c + self.limit_rear) / 2.0

        elif self.current_state == 'SHUNT_BACKWARD_1':
            if not self.plan: return
            control.reverse = True
            control.throttle = 0.2
            control.brake = 0.0
            control.steer = -self.delta_max if self.plan['is_left'] else self.delta_max
            
            if x_c <= self.shunt_mid_x:
                self.current_state = 'SHUNT_BACKWARD_2'
                self.get_logger().info('Switching to SHUNT_BACKWARD_2')

        elif self.current_state == 'SHUNT_BACKWARD_2':
            if not self.plan: return
            control.reverse = True
            control.throttle = 0.2
            control.brake = 0.0
            control.steer = self.delta_max if self.plan['is_left'] else -self.delta_max
            
            yaw_diff = abs(yaw_e - self.center_target_pose[2])
            if x_c <= self.limit_rear or (yaw_diff < math.radians(0.1) and x_c <= self.shunt_mid_x - 0.05):
                control.brake = 1.0
                control.throttle = 0.0
                control.steer = 0.0
                
                self.trials_completed += 1
                if self.trials_completed >= self.total_trials:
                    self.current_state = 'DONE'
                    self.get_logger().info(f"Final position: X={x_c:.2f}, Y={y_c:.2f}")
                    self.get_logger().info('Multiple trials parking complete.')
                else:
                    self.current_state = 'WAIT_BEFORE_FORWARD'
                    self.get_logger().info(f"Shunt loop [{self.trials_completed}/{self.total_trials}] complete. Position: X={x_c:.2f}, Y={y_c:.2f}, Yaw={math.degrees(yaw_e):.2f} deg")
                    
        elif self.current_state == 'WAIT_BEFORE_FORWARD':
            control.brake = 1.0
            control.throttle = 0.0
            if speed < 0.001:
                self.current_state = 'SHUNT_FORWARD_1'
                self.change_gear_times += 1
                self.shunt_mid_x = (x_c + self.limit_front) / 2.0

        elif self.current_state == 'DONE':
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            if not self.plot_triggered:
                self.trigger_plot()
                self.plot_triggered = True
            
        gear_msg = Int32()
        gear_msg.data = self.change_gear_times
        self.gear_pub.publish(gear_msg)

        self.cmd_pub.publish(control)

    def destroy_node(self):
        if hasattr(self, 'log_file_handle') and self.log_file_handle and not self.log_file_handle.closed:
            self.log_file_handle.close()
        super().destroy_node()

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
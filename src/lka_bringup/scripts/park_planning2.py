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

        # Vehicle parameters TESLA Model 3
        self.a = 3.005                                 # distance b/w rear and front axle
        self.b = 1.67/2.0                              # half track width
        self.dFront = 0.81                             # front overhang
        self.dRear = 0.98                              # rear overhang
        self.dSide = 0.25                              # side distance to side mirror
        self.rearAxisOffset = 1.42                     # distance from center to rear axle
        self.steerMax = math.radians(35.0)             # maximum steering angle 
        self.REMin = self.a / math.tan(self.steerMax)  # minimum turning radius

        self.L = 10.0                                  # parking spot length
        self.currentPose = (0.0, 0.0, 0.0)             # car center (x, y, yaw)
        self.targetCenterPose = (70.95, -105.0, 0.0)   # car center (x, y, yaw)
        self.tangentPoint = None                       # tangent point between 1st and 2nd arc
        self.firstSteer = 0.0
        self.isReadOdom = False
        self.state = 'EVALUATING'
        self.speed = 0.0
        self.startTime = None                          # Added state timer latch
        
        self.odom_sub = self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.odomCallback, 10)
        self.cmd_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)

    def getRearAxlePosition(self, carCenterPose):
        xCenter, yCenter, yaw = carCenterPose
        xE = xCenter - self.rearAxisOffset * math.cos(yaw)
        yE = yCenter - self.rearAxisOffset * math.sin(yaw)
        return (xE, yE, yaw)  # Returns heading angle too to clean up packing issues

    def singleConditionCheck(self, cosAlpha, L, dClEInit):
        RBMin = math.sqrt((self.REMin + self.b + self.dSide)**2 + (self.a + self.dFront)**2)
        LMin = self.dRear + math.sqrt(RBMin**2 - (self.REMin - self.b - self.dSide)**2)
        
        # Validated Equation 6 geometry calculation
        dClEInitMin = self.REMin*cosAlpha + math.sqrt((self.REMin*cosAlpha)**2 + self.REMin**2 + 2*self.REMin**2)

        if L >= LMin and dClEInit >= 1.05 * dClEInitMin: # Added 5% simulation safety padding
            return True
        else:
            return False
    
    def getSecoundCircleParking(self, parkPose, EInitPoint):
        xPark, yPark, yawPark = parkPose
        Cx = xPark + self.REMin*math.cos(yawPark + math.pi/2)
        Cy = yPark + self.REMin*math.sin(yawPark + math.pi/2)

        dx = Cx - EInitPoint[0]
        dy = Cy - EInitPoint[1]
        dClEInit = math.hypot(dx, dy)

        return {
            'circleCenter': (Cx, Cy),
            'radius': self.REMin,
            'distanceClEInit': dClEInit
        }
    
    def getFirstCircleParking(self, dClEInit, cosAlpha, EofInitPose):
        num = dClEInit**2 - self.REMin**2
        den = 2*self.REMin + 2*dClEInit*cosAlpha  
        REinit = num / math.fabs(den)
        
        xInit, yInit, yawInit = EofInitPose
        Cx = xInit + REinit * math.cos(yawInit + math.pi/2)
        Cy = yInit + REinit * math.sin(yawInit + math.pi/2)

        return {
            'circleCenter': (Cx, Cy),
            'radius': REinit
        }

    def odomCallback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.currentPose = (x, y, yaw) 

        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.isReadOdom = True
        self.manageTask()

    def getCosAlpha(self, centerSecondArc, EInitPose):
        Cx, Cy = centerSecondArc
        Ex, Ey, EYaw = EInitPose
        dx = Cx - Ex
        dy = Cy - Ey
        ECl = math.hypot(dx, dy)
        dirX = math.cos(EYaw + math.pi/2)
        dirY = math.sin(EYaw + math.pi/2)
        return (dx * dirX + dy * dirY) / ECl

    def planning(self):
        if not self.isReadOdom:
            return 'WAIT'

        rearAxelPosition = self.getRearAxlePosition(self.currentPose)
        
        secondCircle = self.getSecoundCircleParking(self.targetCenterPose, rearAxelPosition)
        Clx, Cly = secondCircle['circleCenter']
        dClEInit = secondCircle['distanceClEInit']

        cosAlpha = self.getCosAlpha(secondCircle['circleCenter'], rearAxelPosition)
 
        if not self.singleConditionCheck(cosAlpha, self.L, dClEInit):
            return 'SEARCH START POINT'
        else:
            firstCircle = self.getFirstCircleParking(dClEInit, cosAlpha, rearAxelPosition)
            Cix, Ciy = firstCircle['circleCenter']
            REInit = firstCircle['radius']
            
            # Left side tracking logic configurations
            self.firstSteer = math.atan2(self.a, REInit)
            
            Tx = Cix + (REInit / (REInit + self.REMin)) * (Clx - Cix)
            Ty = Ciy + (REInit / (REInit + self.REMin)) * (Cly - Ciy)
            self.tangentPoint = (Tx, Ty)

            return 'PARKING FEASIBLE'
        
    def manageTask(self):
        if not self.isReadOdom:
            return

        msg = CarlaEgoVehicleControl()
        msg.hand_brake = False
        
        x_e, y_e, yaw_e = self.getRearAxlePosition(self.currentPose)

        match self.state:
            case 'EVALUATING':
                result = self.planning()
                if result == 'SEARCH START POINT':
                    self.state = 'SEARCHING'
                elif result == 'PARKING FEASIBLE':
                    self.state = 'FIRST STEER'
                    self.startTime = self.get_clock().now()
            
            case 'SEARCHING':
                msg.reverse = False
                msg.throttle = 0.3
                msg.brake = 0.0
                
                if self.planning() == 'PARKING FEASIBLE':
                    msg.throttle = 0.0
                    msg.brake = 1.0
                    self.state = 'FIRST STEER'
                    self.startTime = self.get_clock().now()
            
            case 'FIRST STEER':
                msg.brake = 1.0
                msg.throttle = 0.0
                msg.steer = self.firstSteer
                
                # Lets mechanical gears turn fully before driving backward
                elapsed = (self.get_clock().now() - self.startTime).nanoseconds / 1e9
                if elapsed > 1.5 and self.speed < 0.01:
                    self.state = 'FIRST BACKWARD'

            case 'FIRST BACKWARD':
                msg.brake = 0.0
                msg.throttle = 0.3
                msg.steer = self.firstSteer
                
                dx = x_e - self.tangentPoint[0]
                dy = y_e - self.tangentPoint[1]
                distanceToTangent = math.hypot(dx, dy)
                
                if distanceToTangent < 0.4: # Expanded threshold gap to prevent skip-over
                    self.state = 'SECOND STEER'
                    self.startTime = self.get_clock().now()

            case 'SECOND STEER':
                msg.brake = 1.0
                msg.throttle = 0.0
                msg.steer = -self.steerMax
                
                elapsed = (self.get_clock().now() - self.startTime).nanoseconds / 1e9
                if elapsed > 1.5 and self.speed < 0.01:
                    self.state = 'SECOND BACKWARD'

            case 'SECOND BACKWARD':
                msg.brake = 0.0
                msg.throttle = 0.3
                msg.steer = -self.steerMax
                
                # Check absolute orientation tracking bounds instead of unstable spatial error
                if abs(yaw_e) < math.radians(1.0):
                    self.state = 'PARKED'

            case 'PARKED':
                msg.throttle = 0.0
                msg.brake = 1.0
                msg.hand_brake = True
                msg.steer = 0.0

        self.cmd_pub.publish(msg)

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
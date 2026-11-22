#!/usr/bin/env python3

import math
import rclpy
import time

from std_msgs.msg import String
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        #publisher: sends velocity commands to the robot
        self.cmd_pub = self.create_publisher(
            Twist, #message type
            '/mrp/cmd_vel', #topic name for velocity commands
            10 #queue size 
        )

        #subscriber: receives LIDAR data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback, #update latest_scan with incoming LIDAR data
            10
        )

        #subscriber: receives robot odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/mrp/odom',
            self.odom_callback,
            10
        )

        #service: terminal command for 180 degree turn
        self.turn_service = self.create_service(
            Trigger,
            '/mrp/turn_around',
            self.turn_around_callback 
        )
        self.turn_event_pub = self.create_publisher(
            String,
            '/mrp/turn_event',
            10
        )

        #timer: main control loop
        self.timer = self.create_timer(0.1, self.control_loop) #calls control_loop every 0.1 seconds (10 Hz)

        #sensor values
        self.latest_scan = None
        self.current_yaw = None
        self.current_position = None #store robot's current position for odometry recording

        #motion state
        self.mode = 'NAVIGATE' 

        #turning variables
        self.turn_start_yaw = None
        self.turn_target_angle = 0.0
        self.turn_direction = 1.0 #1.0 for left turn, -1.0 for right turn

        #speed settings
        self.forward_speed = 0.6
        self.turn_speed = 0.7
        self.slow_speed = 0.25 #reduced speed for careful maneuvers

        #distance limits
        self.front_limit = 0.5 #minimum distance before emergency turn
        self.front_slow_limit = 0.75 #distance at which to start slowing down
        self.side_limit = 0.37 #minimum distance to maintain from side walls
        self.side_optimal = 0.62 #ideal distance from side walls for following
        self.corner_distance = 0.4 #distance threshold for detecting corners

        #wall following state
        self.wall_following_side = None #'left' or 'right', determines which wall to follow
        
        #turn timing to prevent oscillation
        self.last_turn_time = 0
        self.turn_cooldown = 1.5 #minimum seconds between turns

        self.get_logger().info('Navigation started')

    #update latest_scan with incoming LIDAR data
    def scan_callback(self, msg):
        self.latest_scan = msg

    #update current_yaw and current_position with incoming odometry data
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation #(x, y, z, w) quaternion orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.current_position = msg.pose.pose.position #store position for bag recording

    def turn_around_callback(self, request, response):
        if self.current_yaw is None:
            response.success = False
            response.message = 'Cannot turn: odometry is not available'
            return response

        self.start_turn(target_angle=math.pi, direction=-1.0) #180 degrees, right turn
        response.success = True
        response.message = 'Started 180 degree turn'
        return response

    #if LIDAR data or odometry is not available, stop the robot. Otherwise, handle turning or navigation based on the current mode
    def control_loop(self):
        if self.latest_scan is None or self.current_yaw is None:
            self.stop_robot()
            return

        if self.mode == 'TURNING':
            self.handle_turning()
        else:
            self.handle_navigation()

    def handle_navigation(self):
        #get distance readings from different sectors for comprehensive environment awareness
        front = self.get_sector_min(-15, 15) #narrow front sector for precise obstacle detection
        front_left = self.get_sector_min(15, 45) #front-left sector for corner detection
        front_right = self.get_sector_min(-45, -15) #front-right sector for corner detection
        left = self.get_sector_min(45, 90) #left side for wall following
        right = self.get_sector_min(-90, -45) #right side for wall following
        far_left = self.get_sector_min(90, 135) #far left for detecting open spaces
        far_right = self.get_sector_min(-135, -90) #far right for detecting open spaces

        cmd = Twist()
        current_time = time.time()

        #if obstacle is critically close in front, perform turn
        if front < self.front_limit:
            self.get_logger().warn(f'Obstacle detected at {front:.2f}m in front')
            
            #check if we are still in cooldown period to prevent rapid consecutive turns
            if current_time - self.last_turn_time < self.turn_cooldown:
                self.stop_robot() #wait for cooldown to expire
                return
            
            #decide turn direction based on which side has more open space
            if left > right:
                turn_angle = math.pi / 2.0 #turn left 90 degrees
                direction = 1.0
            else:
                turn_angle = math.pi / 2.0 #turn right 90 degrees
                direction = -1.0
                
            self.start_turn(target_angle=turn_angle, direction=direction)
            return

        #corner detection: if front is getting close and a side wall is very near
        if front < self.front_slow_limit and (left < self.corner_distance or right < self.corner_distance):
            if left < self.corner_distance:
                #corner on left side detected, turn right to follow the corridor
                self.get_logger().info('Left corner detected, turning right')
                if current_time - self.last_turn_time >= self.turn_cooldown: #check cooldown before turning to prevent oscillation in tight corners
                    self.start_turn(target_angle=math.pi / 2.0, direction=-1.0) 
                    return
            else:
                #corner on right side detected, turn left to follow the corridor
                self.get_logger().info('Right corner detected, turning left')
                if current_time - self.last_turn_time >= self.turn_cooldown:
                    self.start_turn(target_angle=math.pi / 2.0, direction=1.0)
                    return

        #initialize wall following: determine which wall is closer to follow
        if self.wall_following_side is None:
            if left < right and left < 1.0: 
                self.wall_following_side = 'left' #follow left wall
            elif right < left and right < 1.0:
                self.wall_following_side = 'right' #follow right wall
        
        #wall following behavior 
        if self.wall_following_side == 'left':
            if left < self.side_limit:
                #too close to left wall, steer right to move away
                cmd.linear.x = self.slow_speed
                cmd.angular.z = -0.15
            elif left > self.side_optimal + 0.1:
                #too far from left wall, steer left to get closer
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.08
            else:
                #optimal distance maintained, go straight
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
                
            #safety check: also ensure we are not getting too close to the right wall
            if right < self.side_limit:
                cmd.linear.x = self.slow_speed
                cmd.angular.z = 0.1 #steer left to avoid right wall
                
        elif self.wall_following_side == 'right':
            if right < self.side_limit:
                #too close to right wall, steer left to move away
                cmd.linear.x = self.slow_speed
                cmd.angular.z = 0.15
            elif right > self.side_optimal + 0.1:
                #too far from right wall, steer right to get closer
                cmd.linear.x = self.forward_speed
                cmd.angular.z = -0.08
            else:
                #optimal distance maintained, go straight
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
                
            #safety check: also ensure we are not getting too close to the left wall
            if left < self.side_limit:
                cmd.linear.x = self.slow_speed
                cmd.angular.z = -0.1 #steer right to avoid left wall
        else:
            #no wall to follow, go straight
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        #slow down when approaching obstacles in front
        if front < self.front_slow_limit:
            cmd.linear.x = min(cmd.linear.x, self.slow_speed)
            self.get_logger().debug(f'Slowing down: obstacle at {front:.2f}m')

        #switch wall following side if the current wall disappears and another appears
        if self.wall_following_side == 'left' and left > 2.0 and right < 1.0:
            self.wall_following_side = 'right' #switch to right wall
        elif self.wall_following_side == 'right' and right > 2.0 and left < 1.0:
            self.wall_following_side = 'left' #switch to left wall

        self.cmd_pub.publish(cmd)

    def start_turn(self, target_angle, direction):
        self.mode = 'TURNING'
        self.turn_start_yaw = self.current_yaw
        self.turn_target_angle = target_angle
        self.turn_direction = direction
        self.last_turn_time = time.time() #record turn time for cooldown
        
        #reset wall following after turn since the environment perspective changes
        self.wall_following_side = None
        
        #create readable information about the turn
        angle_deg = round(math.degrees(target_angle))

        if direction > 0:
            turn_side = 'left'
        else:
            turn_side = 'right'

        if abs(target_angle - math.pi) < 0.2:
            turn_type = '180'
        else:
            turn_type = '90'

        #publish message for the camera node
        msg = String()
        msg.data = f'turn_{turn_type}_{turn_side}'
        self.turn_event_pub.publish(msg)

        self.get_logger().info(
            f'Starting turn: {math.degrees(target_angle):.1f} degrees {turn_side}'
        )

    def handle_turning(self):
        #compute how much we have turned since starting the turn command
        turned_angle = abs(
            self.shortest_angle_difference(self.turn_start_yaw, self.current_yaw)
        )

        cmd = Twist()

        if turned_angle < self.turn_target_angle - 0.02:
            #still turning, continue rotation
            cmd.linear.x = 0.0
            
            #slow down near the target angle for better accuracy
            remaining = self.turn_target_angle - turned_angle
            if remaining < 0.2:
                cmd.angular.z = self.turn_direction * 0.3 
            else:
                cmd.angular.z = self.turn_direction * self.turn_speed #full speed turn
                
            self.cmd_pub.publish(cmd)
        else:
            #turn completed, stop and return to navigation mode
            self.stop_robot()
            self.mode = 'NAVIGATE'
            self.get_logger().info('Turn finished. Continuing navigation')

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    #get median LIDAR distance in a specified angular sector 
    def get_sector_min(self, angle_min_deg, angle_max_deg):
        
        if self.latest_scan is None:
            return float('inf')

        #convert sector angles from degrees to radians
        angle_min = math.radians(angle_min_deg)
        angle_max = math.radians(angle_max_deg)

        values = []

        for i, distance in enumerate(self.latest_scan.ranges):
            if math.isinf(distance) or math.isnan(distance): #ignore invalid LIDAR readings
                continue

            angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment #e.g -3.14 + 100 * 0.01 = -2.14
            #inside the specified sector, add the distance to the list of values
            if angle_min <= angle <= angle_max:
                values.append(distance)

        if len(values) == 0:
            return float('inf')

        #use median value to be more robust against sensor noise
        values.sort()
        median_idx = len(values) // 2
        return values[median_idx]

    def quaternion_to_yaw(self, x, y, z, w):
       
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def shortest_angle_difference(self, start, current):
        diff = current - start
        #[-pi, pi] range
        while diff > math.pi:
            diff -= 2.0 * math.pi

        while diff < -math.pi:
            diff += 2.0 * math.pi

        return diff


def main(args=None):
    rclpy.init(args=args)

    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from fws_msgs.srv import SetMode
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')

        ## Parameters
        timer_period = 0.02 ## 50Hz
        self.wheel_seperation = 0.63
        self.wheel_base = 0.64
        self.wheel_radius = 0.13
        self.wheel_steering_y_offset = 0.03 ## Not used
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        ## Variables
        self.cmd_vel = None     # robot velocity to be commanded
        self.operation_mode = 0 # Default: 0 Auto, 1: In Phase, 2: Opposite Phase, 3: Pivot Turn

        ## ROS Communication
        self.pub_steer_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_drive_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.srv = self.create_service(SetMode, 'set_mode', self.set_mode_callback)
        self.timer = self.create_timer(timer_period, self.control_cycle)
    
    def set_mode_callback(self, request, response):
        ## Check if mode is valid
        if request.mode != 0 and request.mode != 1 and request.mode != 2:
            response.success = False
            response.message = 'Invalid mode'
            return response
        
        ## Set operation mode
        self.operation_mode = request.mode
        response.success = True
        response.message = 'Mode set to ' +  str(request.mode)
        return response
    
    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
    
    def operation_mode_switch(self):
        ## Order: left_front, right_front, left_rear, right_rear
        pos = np.array([0,0,0,0], float) # For steering Motors
        vel = np.array([0,0,0,0], float) # For driving Motors

        if(self.operation_mode == 1):
            pos,vel = self.in_phase()
        elif(self.operation_mode == 2):
            pos,vel = self.opposite_phase()
        elif(self.operation_mode == 3):
            pos,vel = self.pivot_turn()
        else:
            ## Default: Auto mode
            if self.cmd_vel.linear.x !=0 and self.cmd_vel.linear.y !=0 and self.cmd_vel.angular.z ==0:
                pos,vel = self.in_phase()
            elif self.cmd_vel.linear.x !=0 and self.cmd_vel.linear.y ==0 and self.cmd_vel.angular.z !=0:
                pos,vel = self.opposite_phase()
            elif self.cmd_vel.linear.x ==0 and self.cmd_vel.linear.y ==0 and self.cmd_vel.angular.z !=0:
                pos,vel = self.pivot_turn()
            else:
                pos,vel = self.in_phase()

        return pos,vel
    
    def in_phase(self):
        ## Input: linear_x, linear_y
        pos = np.array([0,0,0,0], float) # For steering Motors
        vel = np.array([0,0,0,0], float) # For driving Motors

        V = math.hypot(self.cmd_vel.linear.x, self.cmd_vel.linear.y)
        sign = np.sign(self.cmd_vel.linear.x)
        
        if(self.cmd_vel.linear.x != 0):
            ang = self.cmd_vel.linear.y / self.cmd_vel.linear.x
        else:
            ang = 0
        
        pos[0] = math.atan(ang)
        pos[1] = math.atan(ang)
        pos[2] = pos[0]
        pos[3] = pos[1]

        ## Position: 0 ~ pi
        pos += math.pi/2

        ## Velocity: 0 ~ 1
        vel[:] = sign*V
        vel[1] *= -1
        vel[3] *= -1        
        return pos,vel
    
    def opposite_phase(self): 
        ## Input: linear_x, angular_z
        pos = np.array([0,0,0,0], float) # For steering Motors
        vel = np.array([0,0,0,0], float) # For driving Motors

        vel_steerring_offset = self.cmd_vel.angular.z * self.wheel_steering_y_offset
        sign = np.sign(self.cmd_vel.linear.x)

        vel[0] = sign*math.hypot(self.cmd_vel.linear.x - self.cmd_vel.angular.z*self.steering_track/2, self.cmd_vel.angular.z*self.wheel_base/2) - vel_steerring_offset
        vel[1] = sign*math.hypot(self.cmd_vel.linear.x + self.cmd_vel.angular.z*self.steering_track/2, self.cmd_vel.angular.z*self.wheel_base/2) + vel_steerring_offset
        vel[2] = sign*math.hypot(self.cmd_vel.linear.x - self.cmd_vel.angular.z*self.steering_track/2, self.cmd_vel.angular.z*self.wheel_base/2) - vel_steerring_offset
        vel[3] = sign*math.hypot(self.cmd_vel.linear.x + self.cmd_vel.angular.z*self.steering_track/2, self.cmd_vel.angular.z*self.wheel_base/2) + vel_steerring_offset

        pos[0] = math.atan(self.cmd_vel.angular.z*self.wheel_base/(2*self.cmd_vel.linear.x + self.cmd_vel.angular.z*self.steering_track)) 
        pos[1] = math.atan(self.cmd_vel.angular.z*self.wheel_base/(2*self.cmd_vel.linear.x - self.cmd_vel.angular.z*self.steering_track))
        pos[2] = -pos[0]
        pos[3] = -pos[1]

        ## Position: 0 ~ pi
        pos += math.pi/2

        ## Velocity: 0 ~ 1
        vel[1] *= -1
        vel[3] *= -1        

        return pos,vel

    def pivot_turn(self):
        ## Input: angular_z
        pos = np.array([0,0,0,0], float) # For steering Motors
        vel = np.array([0,0,0,0], float) # For driving Motors
   
        pos[0] = -math.atan(self.wheel_base/self.steering_track)
        pos[1] = math.atan(self.wheel_base/self.steering_track)
        pos[2] = math.atan(self.wheel_base/self.steering_track)
        pos[3] = -math.atan(self.wheel_base/self.steering_track)
        
        vel[0] = -self.cmd_vel.angular.z
        vel[1] = self.cmd_vel.angular.z
        vel[2] = vel[0]
        vel[3] = vel[1]

        ## Position: 0 ~ pi
        pos += math.pi/2

        ## Velocity: 0 ~ 1
        vel[1] *= -1
        vel[3] *= -1        
        return pos,vel

    def control_cycle(self):
        ## Check if cmd_vel is received
        if self.cmd_vel == None:
            return
        
        ## Get position and velocity commands
        pos,vel = self.operation_mode_switch()

        ## Set position and velocity commands
        steer_cmd = Float64MultiArray(data=pos)
        drive_cmd = Float64MultiArray(data=vel)

        ## Publish commands
        self.pub_steer_pos.publish(steer_cmd)
        self.pub_drive_vel.publish(drive_cmd)

def main(args=None):
    rclpy.init(args=args)
    robot_controller_node = RobotControllerNode()
    rclpy.spin(robot_controller_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

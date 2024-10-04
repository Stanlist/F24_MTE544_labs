# Imports
import rclpy

from utilities import Logger, euler_from_quaternion

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # Velocity message publisher
        self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", 10)
                
        # Loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # QoS Profiler
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # IMU subscription
        self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=qos)
        self.imu_initialized=True
        
        # ENCODER subscription
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=qos)
        self.odom_initialized=True 
        
        # LaserScan subscription 
        self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile=qos)
        self.laser_initialized=True
        
        print("gonna call timer")
        self.create_timer(0.1, self.timer_callback)
        
        # Twist Message Parameters
        self.lin_vel = 0
        self.ang_vel = 0

    def imu_callback(self, imu_msg: Imu):
        # log imu msgs
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds
        imu_ang_vel = imu_msg.angular_velocity.z
        imu_lin_accel_x = imu_msg.linear_acceleration.x
        imu_lin_accel_y = imu_msg.linear_acceleration.y
        self.imu_logger.log_values([imu_lin_accel_x, imu_lin_accel_y,imu_ang_vel, timestamp])
        
    def odom_callback(self, odom_msg: Odometry):
        # log odom msgs
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        odom_orientation = odom_msg.pose.pose.orientation.w
        odom_x_pos = odom_msg.pose.pose.position.x
        odom_y_pos = odom_msg.pose.pose.position.y
        self.odom_logger.log_values([odom_x_pos, odom_y_pos, odom_orientation, timestamp])
                
    def laser_callback(self, laser_msg: LaserScan):
        # log laser msgs with position msg at that time
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds

        ranges = laser_msg.ranges
        angle_increment = laser_msg.angle_increment
        
        self.laser_logger.log_values([ranges, angle_increment, timestamp])
                
    def timer_callback(self):
        print(f"timer callback {self.type}")
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            print("all initialized")
            self.successful_init=True
            
        if not self.successful_init:
            print("failure")
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            print("line if statement")
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        print(cmd_vel_msg)
        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        
        msg=Twist()
        
        # Tweak these values; what range of velocity is appropriate?
        self.lin_vel = 1.0
        self.ang_vel = 6.0
        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        print(msg)
        return msg

    def make_spiral_twist(self):
        msg=Twist()
        
        # Complete a spiral motion by gradually increasing the linear velocity
        # sent to the robot while maintaining constant angular velocity
        self.lin_vel += 0.05
        self.ang_vel = 5.0
        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        self.lin_vel = 6.0
        self.ang_vel = 0.0
        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        print(msg)
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        print("main: go in a line")
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {args.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")

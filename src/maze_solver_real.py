#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

max_lidar_dista = 3.5  
min_lidar_dista = 0.1 
spin = 0.2
class MazeSolverReal:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.dista_from_wall = 0.2
        self.twist = Twist()
        self.front_dista = 0
        
        
        self.front_right = None
        self.right = None
        self.back_right = None

        self.right_dist = None
        while self.right_dist is None:
            continue
        
    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
       #Calculating distance from wall in front, to the left, and to the right
        front = [r for r in range(360) if r not in range(2, 358)]
        right_angles = range(180, 360)
        front_wall_angles = range(298, 302)
        right_wall_angles = range(262, 278)
        back_wall_angles = range(238, 242)


        #plat2 uses 720
        # front = [r for r in range(720) if r not in range(8, 716)]
        # right_angles = range(360, 720)
        # front_wall_angles = range(596, 604)
        # right_wall_angles = range(536, 544)
        # back_wall_angles = range(376, 484)


        front_ranges = [msg.ranges[r] for r in front if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]
        right_ranges = [msg.ranges[r] for r in right_angles if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]

        front_wall = [msg.ranges[r] for r in front_wall_angles if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]
        right_wall = [msg.ranges[r] for r in right_wall_angles if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]
        back_wall = [msg.ranges[r] for r in back_wall_angles if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]
        # print("message ranges", len(msg.ranges))
        self.front_right = min(front_wall) if len(front_wall) > 0 else 5
        self.right = min(right_wall) if len(right_wall) > 0 else 5
        self.back_right = min(back_wall) if len(back_wall) > 0 else 5

        self.right = min(self.right, self.front_right, self.back_right)
        self.front_dista = min(front_ranges) if len(front_ranges) > 0 else None
        self.right_dist = min(right_ranges) if len(right_ranges) > 0 else None


    def follow_wall(self):
        """Makes the robot follow a wall."""
        kp = .774
        kd = 6.4

        #Initializing error
        wall_error = self.dista_from_wall - (self.right if self.right else 0)
        prev_error = wall_error
        d_error = 0

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            wall_error = self.dista_from_wall - (self.right if self.right else 0)
            d_error = wall_error - prev_error if wall_error != prev_error else d_error
            front_error = (self.dista_from_wall) / (self.front_dista + .5)**8  if self.front_dista is not None else 0

            # Adjust angular velocity using PID control and angle adjustment
            turn = (wall_error + front_error ) * kp + d_error * kd
            if (self.back_right < self.front_right - .1) or (self.right < self.front_right -.2):
                turn = -.9

            print (turn)
            self.twist.angular.z = turn
            self.twist.linear.x = .1
            # print("twist val",self.twist)
            self.cmd_vel_pub.publish(self.twist)
            prev_error = wall_error
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('maze_solver_real')
    MazeSolverReal().follow_wall()

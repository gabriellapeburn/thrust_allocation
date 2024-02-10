#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node

#System imports - not always used but necessary sometimes
import sys
import os

#Import the interfaces here
#for example, using the Header msg from the std_msgs package
from geometry_msgs.msg import WrenchStamped
from clearpath_platform_msgs.msg import Drive

#Define your class - use package name with camel case capitalization
class ThrustAllocation(Node):
    #Initialize the class
    def __init__(self):
        #Create the node with whatever name (package name + node)
        super().__init__('force_torque_control thrust_allocation')
        #Initialize imu and wrench objects
        self.wrench_cmd = Wrench()
        #Define the publishers here
        self.cmd_pub_ = self.create_publisher(Drive, '/drive', 10)
        #Define the subscribers here
        self.wrench_sub_ = self.create_subscription(WrenchStamped, '/wrench_cmd', self.wrench_callback, 10)
        #Variable to track the current time
        self.current_time = self.get_clock().now()
        #Set the timer period and define the timer function that loops at the desired rate
        time_period = 1/10
        self.time = self.create_timer(time_period, self.timer_callback)
        self.k = np.zeros(4,4)

    #How fast do we spin the motor
    def vel_update(self):
        #We know the force in x-direction is Fx and the torque about the z-axis is tau_z supplied by motor
        Fx = self.wrench_cmd.force.x
        tau_z = self.wrench_cmd.torque.z
        tau = np.zeros([2,1]) #these need to come from pub sub I think
        tau[0] = Fx
        tau[1] = tau_z
        #We have a matrix of forces of size (n,r)
        T = np.mat['1 1 1 1 1; -1 1 -1 1'] #4 control surefaces with 2 directions (one front back one yaw)
        #K is a diagonal force coeficient matrix... Make a matrix of 0s and K gains along the diagonal
        k = np.zeros(4,4) 
        K = ([1,1,1,1]) #I feel like 1 is a good place to start for a gain but idk
        k = np.fill_diagonal(K)
        K_inv = np.linalg.inv(k)
        #Make the dagger exponent for the inverse geometry
        T_dagger = np.matmul(np.transpose(T), np.linalg.inv(np.matmul(T, np.transpose(T)))) #eqn 12.252 in foss-thorin
        #Make u vector of motor speed we want to solve for
        # self.u = np.array(4,1) #I do not think I need this line
        u = np.matmul(K_inv, np.matmul(T_dagger, tau))
        return u

    #This is the timer function that runs at the desired rate from above
     def timer_callback(self):

         #This is an example on how to create a message,
         #fill in the header information and add some
         #data, then publish
         cmd_msg.mode = Drive()
         u = self.vel_update()
         cmd_msg.drivers = [u(0), u(1)]
         self.cmd_pub_.publish(self.cmd_msg)

         #This is how to keep track of the current time in a ROS2 node
         self.current_time = self.get_clock().now()
         wheel_vel = self.vel_update()

    #Put your callback functions here - these are run whenever the node loops and when there are messages available to process.
    def Wrench_callback(self, msg):
        self.wrench_cmd = self.msg

    #     #This is an example on how to compare the current time
    #     #to the time in a message
    #     #This next line subtracts the two times, converts to nanoseconds
    #     #then puts it in seconds and checks if its greater than 5 seconds
    #     if (self.get_clock().now()-self.current_time).nanoseconds*(10**-9) > 5:
    #         #How to print an info statement
    #         self.get_logger().info('Greater than 5 seconds')
    #         #How to print a warning statement
    #         self.get_logger().warn('Greater than 5 seconds')
    #         #How to print an error message
    #         self.get_logger().error('Greater than 5 seconds')

#This is some boiler plate code that you slap at the bottom
#to make the node work.
#Make sure to update the name of the function and package, etc
#to match your node
def main(args=None):
    rclpy.init(args=args)

    thrust_allocation = ThrustAllocation()
    rclpy.spin(thrust_allocation)
    thrust_allocation.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
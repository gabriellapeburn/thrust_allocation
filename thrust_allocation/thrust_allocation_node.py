#!/usr/bin/env python3

#Python imports
import numpy as np
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
        #Create the node with whatever name (package_name_node)
        super().__init__('thrust_allocation_node') #force_torque_control for the package?
        #Initialize imu and wrench objects
        self.wrench_cmd = WrenchStamped()
        self.wrench_cmd = self.wrench_cmd.wrench 
        #Define the publishers here
        self.cmd_pub_ = self.create_publisher(Drive, 'drive', 10)
        #Define the subscribers here
        self.wrench_sub_ = self.create_subscription(WrenchStamped, 'wrench_cmd', self.wrench_callback, 10)
        #Variable to track the current time
        self.current_time = self.get_clock().now()
        #Set the timer period and define the timer function that loops at the desired rate
        time_period = 1/10
        self.time = self.create_timer(time_period, self.timer_callback)
        self.k = np.zeros((4,4))

        self.header = None


    #How fast do we spin the motor
    def vel_update(self):
        #We know the force in x-direction is Fx and the torque about the z-axis is tau_z supplied by motor
        if self.wrench_cmd.force.x == None:
            Fx = 0.0
        else:
            Fx = self.wrench_cmd.force.x
        
        if self.wrench_cmd.torque.z == None:
            tau_z = 0.0
        else:
            tau_z = self.wrench_cmd.torque.z
        
        tau = np.zeros((2,1)) #these need to come from pub sub I think
        tau[0] = Fx
        tau[1] = tau_z
        # self.get_logger().warn(f'{tau}')
        #We have a matrix of forces of size (n,r)
        #We have a matrix of forces of size (n,r)
        T = np.array([[1, 1, 1, 1], [-1, 1, -1, 1]]) #4 control surefaces with 2 directions (one front back one yaw)
        #K is a diagonal force coeficient matrix... Make a matrix of 0s and K gains along the diagonal
        ka = np.zeros((4,4), int) 
        # print(ka)
        Kb = 1 #[1,1,1,1] #gains might need some tuning help
        np.fill_diagonal(ka, Kb) #REMONDER TO LOOK HERE THIS IS WHERE THE PROBLEM IS BECAUSE IT SPITS OUT NONE
        # print(ka)
        K_inv = np.linalg.inv(ka)
        # print(K_inv)
        #Make the dagger exponent for the inverse geometry
        # print(np.transpose(T))
        # print(np.matmul(T, np.transpose(T)[0]))
        T_dagger = np.matmul(np.transpose(T), np.linalg.inv(np.matmul(T, np.transpose(T)))) #eqn 12.252 in foss-thorin
        # self.get_logger().warn(f'{T_dagger}')
        #Make u vector of motor speed we want to solve for
        # self.u = np.array(4,1) #I do not think I need this line
        u = np.matmul(K_inv, np.matmul(T_dagger, tau))
        return u

    #This is the timer function that runs at the desired rate from above
    def timer_callback(self):

         #This is an example on how to create a message,
         #fill in the header information and add some
         #data, then publish
         cmd_msg = Drive()
         u = self.vel_update()
        #  self.get_logger().warn(f'{u}')
        #  self.get_logger().warn(f'u[{u[0]}], u[{u[0]}]')
         cmd_msg.drivers[0] = u[0]
         cmd_msg.drivers[1] = u[1]
         self.cmd_pub_.publish(cmd_msg)

         #This is how to keep track of the current time in a ROS2 node
         self.current_time = self.get_clock().now()
         wheel_vel = self.vel_update()

    #Put your callback functions here - these are run whenever the node loops and when there are messages available to process.
    def wrench_callback(self, msg):
        self.wrench_cmd = msg.wrench
        self.header = msg.header
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
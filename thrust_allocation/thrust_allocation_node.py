#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node

#System imports - not always used but necessary sometimes
import sys
import os

#Import the interfaces here
#for example, using the Header msg from the std_msgs package
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu

#Define your class - use package name with camel case capitalization
class ThrustAllocation(Node):
    #Initialize the class
    def __init__(self):
        #Create the node with whatever name (package name + node)
        super().__init__('force_torque_control thrust_allocation')
        #Initialize imu and wrench objects
        self.imu = Imu()
        self.wrench_cmd = Wrench()
        #Define the publishers here
        self.vel_pub_ = self.create_publisher(clearpath_platform_msgs/msg/Drive, '/cmd_vel', 10)

        #Define the subscribers here
        self.wrench_sub_ = self.create_subscription(geometry_msgs/msg/WrenchStamped, '/wrench_cmd', self.wrench_callback, 10)

        #Variable to track the current time
        self.current_time = self.get_clock().now()

        #Set the timer period and define the timer function that loops at the desired rate
        time_period = 1/10
        self.time = self.create_timer(time_period, self.timer_callback)

    #How fast do we spin the motor
    def u(self):
        #We know the force in x-direction is Fx and the torque about the z-axis is tau_z supplied by motor
        self.FT = np.matrix(Fx,tau_z)

        #We have a matrix of forces
        self.T = np.matrix(n,r) = [1 0 1 0 1; 0 1 0 1 0; 0 lx1 0 lx2 -ly3]

        #K is a diagonal force coeficient matrix... Make a matrix of 0s and n along the diagonal
        self.k = np.zeros(4,4)
        self.k = np.fill_diagonal(n)

        #make psi
        self.psi = something

        #Make u vector of motor speed we want to solve for
        self.initu = np.array(4,1)
        return u = np.linalg.inv(self.initu) * self.T * self.psi * self.FT

    #This is the timer function that runs at the desired rate from above
    # def timer_callback(self):

    #     #This is an example on how to create a message,
    #     #fill in the header information and add some
    #     #data, then publish
    #     self.twist_msg = TwistStamped()
    #     self.twist_msg.header.stamp = self.get_clock().now().to_msg()
    #     self.twist_msg.twist.linear.x = 10.0
    #     self.twist_pub_.publish(self.twist_msg)

        # #This is how to keep track of the current time in a ROS2 node
        # self.current_time = self.get_clock().now()

    #Put your callback functions here - these are run whenever the node loops and when there are messages available to process.
    def Wrench_callback(self, msg):
        self.wrench = msg

    #Imu callback function
    def Imu_callback(self,msg):
        self.imu = msg

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

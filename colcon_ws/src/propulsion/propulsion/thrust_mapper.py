#!/usr/bin/env python3
"""
Description: Thrust mapper node subscribes to the effort topic, converts the wrench readings to thruster forces,
and then converts the forces to PWM signals and publishes them.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from thrust_mapper_utils import *
from auv_msgs.msg import ThrusterForces, ThrusterMicroseconds
from geometry_msgs.msg import Wrench
import time
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import WrenchStamped

class ThrusterMapper(Node):
    def __init__(self):
        super().__init__('thrust_mapper')

        self.declare_parameter('distance_thruster_thruster_length', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('distance_thruster_thruster_width', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('angle_thruster', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('distance_thruster_middle_length', rclpy.Parameter.Type.DOUBLE)

        self.l = self.get_parameter('distance_thruster_thruster_length').get_parameter_value().double_value
        self.w = self.get_parameter('distance_thruster_thruster_width').get_parameter_value().double_value
        self.alpha = self.get_parameter('angle_thruster').get_parameter_value().integer_value
        self.a = self.get_parameter('distance_thruster_middle_length').get_parameter_value().double_value

        # Retrieve PWM limits from parameters
        self.declare_parameter('thruster_PWM_lower_limit', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('thruster_PWM_upper_limit', rclpy.Parameter.Type.INTEGER)
        self.thruster_lower_limit = self.get_parameter('thruster_PWM_lower_limit').get_parameter_value().integer_value
        self.thruster_upper_limit = self.get_parameter('thruster_PWM_upper_limit').get_parameter_value().integer_value

        self.pub_us = self.create_publisher(ThrusterMicroseconds, "/propusion/microseconds", 1)
        self.pub_forces = self.create_publisher(ThrusterForces, "/propulsion/forces", 1)

        self.T_inv = self.inverse_transformation_matrix()
        
        self.subscription = self.create_subscription(
            Wrench,
            "/controls/effort",
            self.wrench_to_thrust,
            1)
        self.subscription

        #self.add_on_shutdown(self.shutdown)

        self.re_arm()
        time.sleep(2.0)
        #Buffer and Listener for reference frame transformation
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def inverse_transformation_matrix(self):
        T = np.array([
            # SURGE (X)
            [ np.cos(self.alpha), 0, 0, -np.cos(self.alpha), -np.cos(self.alpha), 0, 0,  np.cos(self.alpha)],
            # SWAY (Y)
            [ -np.sin(self.alpha), 0, 0, -np.sin(self.alpha), np.sin(self.alpha), 0, 0, np.sin(self.alpha)],
            # HEAVE (Z)
            [ 0, -1, -1, 0, 0, -1,-1,0],
            # ROLL (X-rotation)
            [0,  self.w/2,self.w/2,0,0, -self.w/2,-self.w/2,0],
            # PITCH (Y-rotation)
            [0,-self.a,self.a,0,0,self.a,-self.a,0],
            # YAW (Z-rotation)
            [ - (self.a*np.sin(self.alpha) - (self.w/2)*np.cos(self.alpha)),  0,  0, + (self.a*np.sin(self.alpha) - (self.w/2)*np.cos(self.alpha)),
            - (self.a*np.sin(self.alpha) - (self.w/2)*np.cos(self.alpha)),  0,  0, + (self.a*np.sin(self.alpha) - (self.w/2)*np.cos(self.alpha)) ]
        ])
        T_inv = np.linalg.pinv(T)
        return T_inv
    
    def wrench_to_thrust(self, wrench_msg):
        """
        Callback function that maps a received Wrench message into thruster forces.
        It first converts the wrench from global to body frame, then applies the
        pseudo-inverse of the thruster mapping matrix. We assume that all messages published on /controls/effort
        are in the "auv" frame.

        """

        wrench_stamped = WrenchStamped()
        wrench_stamped.header.stamp = Clock().now().to_msg()
        wrench_stamped.header.frame_id = "auv"
        wrench_stamped.wrench = wrench_msg
        
        
        # Construct the 6x1 vector from the body wrench
        a_vec = np.array([
            [wrench_stamped.wrench.force.x],
            [wrench_stamped.wrench.force.y],
            [wrench_stamped.wrench.force.z],
            [wrench_stamped.wrench.torque.x],
            [wrench_stamped.wrench.torque.y],
            [wrench_stamped.wrench.torque.z]
        ])
        
        # Calculate the thruster forces using the pseudo-inverse
        converted_w = np.matmul(self.T_inv, a_vec)
        converted_w = (converted_w.flatten()).reshape((8,1))

        tf_msg = ThrusterForces()
        tf_msg.back_left = converted_w[0][0]
        tf_msg.heave_back_left = converted_w[1][0]
        tf_msg.heave_front_left = converted_w[2][0]
        tf_msg.front_left = converted_w[3][0]
        tf_msg.front_right = converted_w[4][0]
        tf_msg.heave_front_right = converted_w[5][0]
        tf_msg.heave_back_right = converted_w[6][0]
        tf_msg.back_right = converted_w[7][0]
        
        # Publish the computed thruster forces (useful for simulation/debugging)
        self.pub_forces.publish(tf_msg)
        
        # Convert forces to PWM signals and publish them
        self.forces_to_pwm_publisher(tf_msg)
    
    def forces_to_pwm_publisher(self, forces_msg):
        """
        Converts thruster forces into PWM signals and publishes them.
        Applies individual limits to prevent overcurrent.
        """
        pwm_arr = [None] * 8
        pwm_arr[ThrusterMicroseconds.BACK_LEFT] = force_to_pwm_thruster(1,forces_msg.back_left * thruster_mount_dirs[ThrusterMicroseconds.BACK_LEFT])
        pwm_arr[ThrusterMicroseconds.HEAVE_BACK_LEFT] = force_to_pwm_thruster(2,forces_msg.heave_back_left * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_BACK_LEFT])
        pwm_arr[ThrusterMicroseconds.HEAVE_FRONT_LEFT] = force_to_pwm_thruster(3,forces_msg.heave_front_left * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_FRONT_LEFT])
        pwm_arr[ThrusterMicroseconds.FRONT_LEFT] = force_to_pwm_thruster(4,forces_msg.front_left * thruster_mount_dirs[ThrusterMicroseconds.FRONT_LEFT])
        pwm_arr[ThrusterMicroseconds.FRONT_RIGHT] = force_to_pwm_thruster(5,forces_msg.front_right * thruster_mount_dirs[ThrusterMicroseconds.FRONT_RIGHT])
        pwm_arr[ThrusterMicroseconds.HEAVE_FRONT_RIGHT] = force_to_pwm_thruster(6,forces_msg.heave_front_right * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_FRONT_RIGHT])
        pwm_arr[ThrusterMicroseconds.HEAVE_BACK_RIGHT] = force_to_pwm_thruster(7,forces_msg.heave_back_right * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_BACK_RIGHT])
        pwm_arr[ThrusterMicroseconds.BACK_RIGHT] = force_to_pwm_thruster(8,forces_msg.back_right * thruster_mount_dirs[ThrusterMicroseconds.BACK_RIGHT])

        # Apply limit checking for each thruster
        for i in range(len(pwm_arr)):
            if pwm_arr[i] > self.thruster_upper_limit:
                pwm_arr[i] = self.thruster_upper_limit
                self.get_logger().warn(f"INDIVIDUAL FUSE EXCEEDED: Thruster {i + 1}")
            elif pwm_arr[i] < self.thruster_lower_limit:
                pwm_arr[i] = self.thruster_lower_limit
                self.get_logger().warn(f"INDIVIDUAL FUSE EXCEEDED: Thruster {i + 1}")
    
        pwm_msg = ThrusterMicroseconds()
        pwm_msg.microseconds = pwm_arr
        self.pub_us.publish(pwm_msg)
    
    def re_arm(self):
        """
        Sends the arming signal to the thrusters upon startup.
        """
        time.sleep(1)
        msg1 = ThrusterMicroseconds()
        msg1.microseconds = [1500] * 8
        msg2 = ThrusterMicroseconds()
        msg2.microseconds = [1540] * 8
    
        self.pub_us.publish(msg1)
        time.sleep(0.5)
        self.pub_us.publish(msg2)
        time.sleep(0.5)
        self.pub_us.publish(msg1)
    
    def shutdown(self):
        """
        Turns off the thrusters when the node is shutting down.
        """
        msg = ThrusterMicroseconds()
        msg.microseconds = [1500] * 8
        self.pub_us.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)

    thrust_mapper = ThrusterMapper()

    rclpy.spin(thrust_mapper)

    thrust_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

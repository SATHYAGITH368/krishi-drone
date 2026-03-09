#!/usr/bin/env python3
'''
# Team ID:          1002
# Theme:            Krishi Drone
# Author List:      Sathyagith, Rudra, Shavya, Nagendra
# Filename:         pico_server.py
# Functions:        __init__, disarm, arm, whycon_callback, odometry_callback, altitude_set_pid, pitch_set_pid, roll_set_pid, pid_control_callback, execute_callback, main
# Global variables: None
'''

import time
import math
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from waypoint_navigation.action import NavToWaypoint
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from error_msg.msg import Error
from controller_msg.msg import PIDTune
from nav_msgs.msg import Odometry


class WayPointServer(Node):
    '''
    Purpose:
    ---
    Implements a ROS 2 Action Server node that navigates the Swift Pico Drone 
    to the given waypoints using PID control. It processes WhyCon position data, 
    filters it, computes PID control outputs, and publishes motor commands.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    waypoint_server = WayPointServer()
    '''
    def __init__(self):
        '''
        Purpose:
        ---
        Initializes the WayPointServer node, publishers, subscribers, PID parameters,
        filters, and sets up the action server for waypoint navigation.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None
        '''
        super().__init__('waypoint_server')
        self.flag = False
     
        # Callback groups for multi-threaded operation
        self.pid_or_lqr_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        self.odometry_callback_group = ReentrantCallbackGroup()

    
        self.filtered_state = [0.0, 0.0, 0.0]  # filtered position data [x, y, z]
        self.filter_alpha = 0.25              
        self.filter_initialized = False
  

        # Drone state variables
        self.curr_state = [0.0, 0.0, 0.0]
        self.desired_state = [0.0, 0.0, 0.0]
        self.target_state = [0.0, 0.0, 0.0]

        # Drone command message
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1000

        # PID gain values
        self.Kp = [2, 3, 25.0]
        self.Ki = [0.0, 0.0, 0.03]
        self.Kd = [20.0, 20.0, 600]

        # Error storage
        self.alt_error = [0.0, 0.0, 0.0]
        self.prev_alt_error = [0.0, 0.0, 0.0]
        self.sum_alt_error = [0.0, 0.0, 0.0]

        # Filtered derivative terms
        self.prev_derivative = [0.0, 0.0, 0.0]

        # Control flags
        self.control_active = False
        self.sample_time = 0.02
        self.last_debug_time = time.time()
        self.time_inside_sphere = 0.0
        self.point_in_sphere_start_time = None
        self.first_hover_complete = False

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pid_error', 10)

        # Subscribers
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 10)
        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

        # Action Server initialization
        self.action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            execute_callback=self.execute_callback,
            callback_group=self.action_callback_group
        )

     
        self.arm()
        self.timer = self.create_timer(self.sample_time, self.pid_control_callback)

    def disarm(self):
        '''
        Purpose:
        ---
        Disarms the drone by setting all RC channels to 1000 and publishing the command.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None
        '''
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        '''
        Purpose:
        ---
        Arms the drone by setting RC channels to neutral positions 
        and enabling AUX4 for flight control.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None
        '''
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def whycon_callback(self, msg):
        '''
        Purpose:
        ---
        Processes the WhyCon pose data, applies a low-pass filter 
        to reduce noise, and updates the current position.

        Input Arguments:
        ---
        `msg` :  [ PoseArray ]
            The incoming WhyCon position data.

        Returns:
        ---
        None
        '''
        if not msg.poses:
            return

        raw_x = msg.poses[0].position.x
        raw_y = msg.poses[0].position.y
        raw_z = msg.poses[0].position.z

        # Initialize filter with first valid reading
        if not self.filter_initialized:
            self.filtered_state = [raw_x, raw_y, raw_z]
            self.filter_initialized = True

        # Apply exponential moving average filter better than normal low pass filter
        a = self.filter_alpha
        self.filtered_state[0] = a * raw_x + (1 - a) * self.filtered_state[0]
        self.filtered_state[1] = a * raw_y + (1 - a) * self.filtered_state[1]
        self.filtered_state[2] = a * raw_z + (1 - a) * self.filtered_state[2]

        # Update filtered position
        self.curr_state[0] = self.filtered_state[0]
        self.curr_state[1] = self.filtered_state[1]
        self.curr_state[2] = self.filtered_state[2]

        self.dtime = time.time()

    def odometry_callback(self, msg):
        '''
        Purpose:
        ---
        Extracts drone orientation from odometry data and converts it into yaw angle.

        Input Arguments:
        ---
        `msg` :  [ Odometry ]
            Odometry data from the simulation or sensors.

        Returns:
        ---
        None
        '''
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.yaw = math.degrees(yaw)

    def altitude_set_pid(self, alt):
        '''
        Purpose:
        ---
        Updates the PID gains for altitude (throttle) control dynamically.

        Input Arguments:
        ---
        `alt` :  [ PIDTune ]
            PID tuning message containing updated Kp, Ki, and Kd.

        Returns:
        ---
        None
        '''
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.6

    def pitch_set_pid(self, pitch):
        '''
        Purpose:
        ---
        Updates the PID gains for pitch control dynamically.

        Input Arguments:
        ---
        `pitch` :  [ PIDTune ]
            PID tuning message containing updated Kp, Ki, and Kd.

        Returns:
        ---
        None
        '''
        self.Kp[1] = pitch.kp * 0.03
        self.Ki[1] = pitch.ki * 0.008
        self.Kd[1] = pitch.kd * 0.6

    def roll_set_pid(self, roll):
        '''
        Purpose:
        ---
        Updates the PID gains for roll control dynamically.

        Input Arguments:
        ---
        `roll` :  [ PIDTune ]
            PID tuning message containing updated Kp, Ki, and Kd.

        Returns:
        ---
        None
        '''
        self.Kp[0] = roll.kp * 0.03
        self.Ki[0] = roll.ki * 0.008
        self.Kd[0] = roll.kd * 0.04

    def pid_control_callback(self):
        '''
        Purpose:
        ---
        Executes the PID control loop at fixed intervals, computes PID output 
        for roll, pitch, and throttle, and publishes drone commands.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None
        '''
        if not self.control_active:
            return

     #ramping 
        ramp_rate = 0.05
        for i in [0, 1]:
            error = self.target_state[i] - self.desired_state[i]
            if abs(error) > ramp_rate:
                self.desired_state[i] += ramp_rate * (1 if error > 0 else -1)
            else:
                self.desired_state[i] = self.target_state[i]
        self.desired_state[2] = self.target_state[2]

     
        Kp_roll = self.Kp[0] if self.first_hover_complete else 0.0
        Ki_roll = self.Ki[0] if self.first_hover_complete else 0.0
        Kd_roll = self.Kd[0] if self.first_hover_complete else 0.0

        Kp_pitch = self.Kp[1] if self.first_hover_complete else 0.0
        Ki_pitch = self.Ki[1] if self.first_hover_complete else 0.0
        Kd_pitch = self.Kd[1] if self.first_hover_complete else 0.0

        # Compute positional error
        self.alt_error = [self.desired_state[i] - self.curr_state[i] for i in range(3)]

        # Compute filtered derivative term
        raw_derivative = [self.alt_error[i] - self.prev_alt_error[i] for i in range(3)]
        derivative = [0.5 * raw_derivative[i] + 0.5 * self.prev_derivative[i] for i in range(3)]
        self.prev_derivative = derivative

        # PID Control outputs for Roll, Pitch, and Throttle
        self.cmd.rc_pitch = int(1500 + (
            Kp_pitch * self.alt_error[0] +
            Ki_pitch * self.sum_alt_error[0] -
            Kd_pitch * derivative[0]
        ))
        self.cmd.rc_roll = int(1500 + (
            Kp_roll * self.alt_error[1] +
            Ki_roll * self.sum_alt_error[1] +
            Kd_roll * derivative[1]
        ))
        self.cmd.rc_throttle = int(1500 - (
            self.Kp[2] * self.alt_error[2] +
            self.Ki[2] * self.sum_alt_error[2] +
            self.Kd[2] * derivative[2]
        ))

        # Limit command range
        self.cmd.rc_roll = max(1000, min(2000, self.cmd.rc_roll))
        self.cmd.rc_pitch = max(1000, min(2000, self.cmd.rc_pitch))
        self.cmd.rc_throttle = max(1000, min(2000, self.cmd.rc_throttle))

        # Integrate error and update
        for i in range(3):
            self.sum_alt_error[i] += self.alt_error[i]
            if i in [0, 1]:
                self.sum_alt_error[i] = max(-50.0, min(50.0, self.sum_alt_error[i]))
            self.prev_alt_error[i] = self.alt_error[i]

        # Publish command and error for debugging
        self.command_pub.publish(self.cmd)
        err = Error()
        err.roll_error, err.pitch_error, err.throttle_error = map(float, self.alt_error)
        err.yaw_error = 0.0
        self.pos_error_pub.publish(err)

       
        now = time.time()
        if now - self.last_debug_time >= 0.5:
            if abs(self.alt_error[0]) > 1.0:
                self.get_logger().warn(f"Large X (Pitch) Error: {self.alt_error[0]:.2f}")
            if abs(self.alt_error[1]) > 1.0:
                self.get_logger().warn(f"Large Y (Roll) Error: {self.alt_error[1]:.2f}")
            self.last_debug_time = now

    def execute_callback(self, goal_handle):
        '''
        Purpose:
        ---
        Executes waypoint navigation when a goal is received from the Action Client.
        Maintains hover for a set duration when inside a target sphere.

        Input Arguments:
        ---
        `goal_handle` :  [ ActionServer GoalHandle ]
            Contains the waypoint target for the drone.

        Returns:
        ---
        `result` :  [ NavToWaypoint.Result ]
            Action result message containing hover time at waypoint.
        '''
        self.get_logger().info('Executing new waypoint goal...')

        # Adjust PID gains for subsequent flights
        if self.flag:
            self.Kp = [1.0, 2.0, 20.0]
            self.Ki = [0.0, 0.0, 0.012]
            self.Kd = [15.0, 30.0, 500.0]
            radius = 0.8
        else:
            radius = 0.8

        self.flag = True

        # Set target position
        self.target_state[0] = goal_handle.request.waypoint.position.x
        self.target_state[1] = goal_handle.request.waypoint.position.y
        self.target_state[2] = goal_handle.request.waypoint.position.z
        self.get_logger().info(f"New Target: {self.target_state} | Radius: {radius}")

        self.control_active = True
        feedback_msg = NavToWaypoint.Feedback()
        result = NavToWaypoint.Result()

        # hover timers
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0.0
        self.max_time_inside_sphere = 0.0
        self.duration = time.time()

        hover_time_required = 2.0

        # Action server feedback loop
        while rclpy.ok():
            feedback_msg.current_waypoint.pose.position.x = self.curr_state[0]
            feedback_msg.current_waypoint.pose.position.y = self.curr_state[1]
            feedback_msg.current_waypoint.pose.position.z = self.curr_state[2]
            goal_handle.publish_feedback(feedback_msg)

            current_time = time.time()
            distance_sq = (
                (self.curr_state[0] - self.target_state[0]) ** 2 +
                (self.curr_state[1] - self.target_state[1]) ** 2 +
                (self.curr_state[2] - self.target_state[2]) ** 2
            )
            in_sphere = distance_sq <= radius ** 2

            # Time tracking when inside the hover sphere
            if in_sphere:
                if self.point_in_sphere_start_time is None:
                    self.point_in_sphere_start_time = current_time
                self.time_inside_sphere = current_time - self.point_in_sphere_start_time
            else:
                if self.point_in_sphere_start_time is not None and (current_time - self.point_in_sphere_start_time) < 1.0:
                    pass
                else:
                    self.point_in_sphere_start_time = None
                    self.time_inside_sphere = 0.0

         
            if self.time_inside_sphere > self.max_time_inside_sphere:
                self.max_time_inside_sphere = self.time_inside_sphere

            # Mark first hover as complete
            if not self.first_hover_complete and self.max_time_inside_sphere >= hover_time_required:
                self.first_hover_complete = True

            # Exit when hover time achieved
            if self.time_inside_sphere >= hover_time_required:
                break

            time.sleep(0.02)

        self.control_active = False
        goal_handle.succeed()
        result.hov_time = int(time.time() - self.duration)
        return result


def main(args=None):
    '''
    Purpose:
    ---
    Initializes and runs the waypoint server node with multi-threaded executor.

    Input Arguments:
    ---
    `args` :  [ list ]
        Command line arguments for ROS 2 node initialization.

    Returns:
    ---
    None

    Example call:
    ---
    main()
    '''
    rclpy.init(args=args)
    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        waypoint_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3

'''
# Team ID:          1002
# Theme:            Swift Pico Drone Control
# Author List:      sathyagith, rudra, shavya, nagendra
# Filename:         KD_1002_pico_controller.py
# Functions:        __init__, disarm, arm, whycon_callback, altitude_set_pid, pitch_set_pid, roll_set_pid, pid, main
# Global variables: None
'''

# Importing required libraries
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from controller_msg.msg import PIDTune
from error_msg.msg import Error
import rclpy
from rclpy.node import Node


class Swift_Pico(Node):
    '''
    Purpose:
    ---
    ROS2 Node class for controlling the Swift Pico Drone using PID.
    Publishes drone commands and PID errors, and subscribes to position and PID tuning topics.
    '''

    def __init__(self):
        '''
        Purpose:
        ---
        Initializes the ROS node, sets up publishers/subscribers, and initializes PID variables and control flags.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        swift_pico = Swift_Pico()
        '''
        super().__init__('pico_controller')  # Initialize ROS node

        # Current position of drone [x, y, z]
        self.drone_position = [0.0, 0.0, 0.0]  # [x, y, z]

        # Desired setpoint [x, y, z]
        self.setpoint = [-7, 0, 20]

        # Command message initialization
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1000

        # PID constants [roll, pitch, throttle]
        self.Kp = [0, 0, 68]
        self.Ki = [0, 0, 0.03]
        self.Kd = [0, 0, 955]

        # Error variables
        self.alt_error = [0, 0, 0]
        self.prev_alt_error = [0, 0, 0]
        self.sum_alt_error = [0, 0, 0]

        # Control flags
        self.prev1 = 0  # Pitch control flag
        self.prev2 = 0  # Roll control flag

        # PID loop sample time
        self.sample_time = 0.033  # seconds

        # ROS publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pid_error', 10)

        # ROS subscribers
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

        # Arm the drone initially
        self.arm()

        # Timer to run PID loop periodically
        self.timer = self.create_timer(self.sample_time, self.pid)

    def disarm(self):
        '''
        Purpose:
        ---
        Disarms the drone by setting all RC channels to minimal safe values.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        swift_pico.disarm()
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
        Arms the drone by first disarming (safety) and then setting aux4 high.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        swift_pico.arm()
        '''
        self.disarm()  # Safety disarm
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
        Updates the drone's current position from WhyCon PoseArray messages.

        Input Arguments:
        ---
        `msg` :  [ geometry_msgs.msg.PoseArray ]
            Position message containing the drone's current x, y, z coordinates

        Returns:
        ---
        None

        Example call:
        ---
        Triggered automatically on /whycon/poses topic update
        '''
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_set_pid(self, alt):
        '''
        Purpose:
        ---
        Updates PID constants for altitude control from /throttle_pid topic.

        Input Arguments:
        ---
        `alt` :  [ controller_msg.msg.PIDTune ]
            Contains kp, ki, kd values for throttle/altitude

        Returns:
        ---
        None

        Example call:
        ---
        Triggered automatically on /throttle_pid topic update
        '''
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.6

    def pitch_set_pid(self, pitch):
        '''
        Purpose:
        ---
        Updates PID constants for pitch control from /pitch_pid topic.

        Input Arguments:
        ---
        `pitch` :  [ controller_msg.msg.PIDTune ]
            Contains kp, ki, kd values for pitch

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
        Updates PID constants for roll control from /roll_pid topic.

        Input Arguments:
        ---
        `roll` :  [ controller_msg.msg.PIDTune ]
            Contains kp, ki, kd values for roll

        Returns:
        ---
        None
        '''
        self.Kp[0] = roll.kp * 0.03
        self.Ki[0] = roll.ki * 0.008
        self.Kd[0] = roll.kd * 0.6

    def pid(self):
        '''
        Purpose:
        ---
        Main PID control loop.
        Calculates PID outputs for throttle, pitch, and roll based on setpoints and current drone position.
        Limits command values to safe ranges and publishes drone commands and PID errors.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        Automatically called periodically by ROS2 timer
        '''
        # Calculate positional errors
        self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
        self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
        self.alt_error[1] = self.setpoint[1] - self.drone_position[1]

        # PID computation for throttle
        self.cmd.rc_throttle = int(1500 - self.Kp[2] * self.alt_error[2]
                                   - self.Ki[2] * self.sum_alt_error[2]
                                   - self.Kd[2] * (self.alt_error[2] - self.prev_alt_error[2]))

        # PID computation for pitch (with altitude dependency)
        if self.prev1 or abs(self.alt_error[2]) < 15:
            self.prev1 = 1
            self.cmd.rc_pitch = int(1500 - self.Kp[1] * self.alt_error[1]
                                    - self.Ki[1] * self.sum_alt_error[1]
                                    - self.Kd[1] * (self.alt_error[1] - self.prev_alt_error[1]))

        # PID computation for roll (with pitch dependency)
        if self.prev2 or abs(self.alt_error[1]) < 1.5:
            self.prev2 = 1
            self.cmd.rc_roll = int(1500 + self.Kp[0] * self.alt_error[0]
                                   + self.Ki[0] * self.sum_alt_error[0]
                                   + self.Kd[0] * (self.alt_error[0] - self.prev_alt_error[0]))

        # Limit RC channel values to [1000, 2000]
        self.cmd.rc_throttle = max(1000, min(2000, self.cmd.rc_throttle))
        self.cmd.rc_roll = max(1000, min(2000, self.cmd.rc_roll))
        self.cmd.rc_pitch = max(1000, min(2000, self.cmd.rc_pitch))

        # Debugging output
        print(f"Altitude error: {self.alt_error[2]:.2f}")

        # Update error history for integral and derivative terms
        self.prev_alt_error = self.alt_error.copy()
        for i in range(3):
            self.sum_alt_error[i] += self.alt_error[i]

        # Publish drone commands
        self.command_pub.publish(self.cmd)

        # Publish PID errors
        self.pos_error_pub.publish(Error(
            roll_error=float(self.alt_error[0]),
            pitch_error=float(self.alt_error[1]),
            throttle_error=float(self.alt_error[2]),
            yaw_error=0.0
        ))


def main(args=None):
    '''
    Purpose:
    ---
    Initializes ROS2, creates Swift_Pico node, and spins until interrupted.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    Called automatically by OS if script is run
    '''
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    try:
        rclpy.spin(swift_pico)
    except KeyboardInterrupt:
        swift_pico.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        swift_pico.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


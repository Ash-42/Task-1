#!/usr/bin/env python2

import math
import time
import rospy
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

class ControlSystem():

    def __init__(self):

        rospy.init_node('position_controller')

        # [latitude, longitude, altitude]
        self.position_gps = [0.0, 0.0, 0.0]
        self.setpoint_gps = [19.0001, 72.0000, 1]

        # [roll, pitch, altitude]
        self.Kp = [0, 0, 0]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 0]

        self.prev_pid_errors = [0.0, 0.0, 0.0]
        self.iTerm = [0.0, 0.0, 0.0]
        self.outputs = [0.0, 0.0, 0.0]
        self.min_value = 1000
        self.max_value = 2000
        self.sample_time = 0.060

        # Outputs in the range [1000, 2000] to the attitude_controller
        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcRoll = 0.0
        self.drone_cmd.rcPitch = 0.0
        self.drone_cmd.rcYaw = 0.0
        self.drone_cmd.rcThrottle = 0.0

        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)

        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.alt_set_pid)

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3

    def alt_set_pid(self, alt):
        self.Kp[2] = alt.Kp * 0.06
        self.Ki[2] = alt.Ki * 0.008
        self.Kd[2] = alt.Kd * 0.3

    def calculate_pos_errors(self, errors):
        errors[0] = self.setpoint_gps[0] - self.position_gps[0]
        errors[1] = self.setpoint_gps[1] - self.position_gps[1]
        errors[2] = self.setpoint_gps[2] - self.position_gps[2]

    def world_to_body_conversion(self, gps_errors, pid_errors):
        lat_error, long_error, alt_error = [*gps_errors]

        tan_roll = alt_error / lat_error
        tan_pitch = long_error / alt_error
        tan_yaw = lat_error / long_error

        pid_errors[0] = math.degrees(math.atan(tan_roll))
        pid_errors[1] = math.degrees(math.atan(tan_pitch))
        pid_errors[2] = math.degrees(math.atan(tan_yaw))

    def calculate_pid_eq(self, errors):
        pTerm = [self.Kp[0] * errors[0],
                 self.Kp[1] * errors[1],
                 self.Kp[2] * errors[2]]

        dTerm = [self.Kd[0] * (errors[0] - self.prev_pid_errors[0]),
                 self.Kd[1] * (errors[1] - self.prev_pid_errors[1]),
                 self.Kd[2] * (errors[2] - self.prev_pid_errors[2])]

        self.iTerm[0] = self.Ki[0] * (self.iTerm[0] + errors[0])
        self.iTerm[1] = self.Ki[1] * (self.iTerm[1] + errors[1])
        self.iTerm[2] = self.Ki[2] * (self.iTerm[2] + errors[2])

        self.outputs[0] = pTerm[0] + self.iTerm[0] + dTerm[0]
        self.outputs[1] = pTerm[1] + self.iTerm[1] + dTerm[1]
        self.outputs[2] = pTerm[2] + self.iTerm[2] + dTerm[2]

    def set_prev_vals(self, errors):
        self.prev_pid_errors[0] = errors[0]
        self.prev_pid_errors[1] = errors[1]
        self.prev_pid_errors[2] = errors[2]

    def set_cmd_bounds(self, euler_angle):
        if euler_angle > self.max_value:
            return self.max_value
        if euler_angle < self.min_value:
            return self.min_value
        return euler_angle

    def set_bounds(self):
        outputs = [self.outputs[0],
                   self.outputs[1],
                   self.outputs[2]]
        [self.outputs[0],
         self.outputs[1],
         self.outputs[2]] = outputs = map(set_cmd_bounds, outputs)

    def publish_attitude(self):
        self.drone_cmd.rcRoll = self.outputs[0]
        self.drone_cmd.rcPitch = self.outputs[1]
        self.drone_cmd.rcYaw = 1500
        self.drone_cmd.rcThrottle = self.outputs[2]
        self.drone_pub.publish(self.drone_cmd)

    def pid(self):
        gps_errors = [0.0, 0.0, 0.0]
        pid_errors = [0.0, 0.0, 0.0]
        self.calculate_pos_errors(gps_errors)
        self.world_to_body_conversion(gps_errors, pid_errors)
        self.calculate_pid_eq(pid_errors)
        self.set_prev_vals(pid_errors)
        self.set_bounds()
        self.publish_attitude()

if __name__ == '__main__':
    SYSTEM = ControlSystem()
    RATE = rospy.Rate(1 / SYSTEM.sample_time)
    while not rospy.is_shutdown():
        SYSTEM.pid()
        RATE.sleep()

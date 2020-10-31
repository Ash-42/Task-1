#!/usr/bin/env python2

import rospy
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32

class ControlSystem():

    def __init__(self):

        rospy.init_node('position_controller')

        # [x, y, z]
        self.position = [0.0, 0.0, 0.0]
        self.position_stpnt = [19.0, 72.0, 3.0]

        # [x, y, z]
        self.Kp = [0, 0, 720]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 4]

        self.prev_errors = [0.0, 0.0, 0.0]
        self.iTerm = [0.0, 0.0, 0.0]
        self.outputs = [0.0, 0.0, 0.0]
        self.min_value = 1000
        self.max_value = 2000
        self.sample_time = 0.05

        # Outputs in the range [1000, 2000] to the attitude_controller
        self.drone_cmd = edrone_cmd()
        self.drone_cmd.rcRoll = 0.0
        self.drone_cmd.rcPitch = 0.0
        self.drone_cmd.rcYaw = 0.0
        self.drone_cmd.rcThrottle = 0.0

        self.drone_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.x_error_pub = rospy.Publisher('/x_error', Float32, queue_size=1)
        self.y_error_pub = rospy.Publisher('/y_error', Float32, queue_size=1)
        self.z_error_pub = rospy.Publisher('/z_error', Float32, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)

        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.range_callback)
        rospy.Subscriber('/pid_tuning_x', PidTune, self.x_set_pid)
        rospy.Subscriber('/pid_tuning_y', PidTune, self.y_set_pid)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.z_set_pid)

    def gps_callback(self, msg):
        self.position[0] = msg.latitude
        self.position[1] = msg.longitude

    def range_callback(self, msg):
        self.position[2] = msg.ranges[0]

    def x_set_pid(self, x):
        self.Kp[0] = x.Kp * 0.5
        self.Ki[0] = x.Ki * 0.008
        self.Kd[0] = x.Kd * 0.05

    def y_set_pid(self, y):
        self.Kp[1] = y.Kp * 0.5
        self.Ki[1] = y.Ki * 0.008
        self.Kd[1] = y.Kd * 0.05

    def z_set_pid(self, z):
        self.Kp[2] = z.Kp * 0.5
        self.Ki[2] = z.Ki * 0.008
        self.Kd[2] = z.Kd * 0.05

    def calculate_errors(self, errors):
        errors[0] = self.position_stpnt[0] - self.position[0]
        errors[1] = self.position_stpnt[1] - self.position[1]
        errors[2] = self.position_stpnt[2] - self.position[2]

    def calculate_pid_eq(self, errors):
        pTerm = [self.Kp[0] * errors[0],
                 self.Kp[1] * errors[1],
                 self.Kp[2] * errors[2]]

        dTerm = [self.Kd[0] * (errors[0] - self.prev_errors[0]),
                 self.Kd[1] * (errors[1] - self.prev_errors[1]),
                 self.Kd[2] * (errors[2] - self.prev_errors[2])]

        self.iTerm[0] = self.Ki[0] * (self.iTerm[0] + errors[0])
        self.iTerm[1] = self.Ki[1] * (self.iTerm[1] + errors[1])
        self.iTerm[2] = self.Ki[2] * (self.iTerm[2] + errors[2])

        self.outputs[0] = pTerm[0] + self.iTerm[0] + dTerm[0]
        self.outputs[1] = pTerm[1] + self.iTerm[1] + dTerm[1]
        self.outputs[2] = pTerm[2] + self.iTerm[2] + dTerm[2]

    def set_prev_vals(self, errors):
        self.prev_errors[0] = errors[0]
        self.prev_errors[1] = errors[1]
        self.prev_errors[2] = errors[2]

    def set_cmd_bounds(self, rc_outputs):
        if rc_outputs > self.max_value:
            return self.max_value
        if rc_outputs < self.min_value:
            return self.min_value
        return rc_outputs

    def set_bounds(self):
        outputs = [i for i in self.outputs]
        [self.outputs[0],
         self.outputs[1],
         self.outputs[2]] = map(self.set_cmd_bounds, outputs)

    def publish_values(self, errors):
        self.drone_cmd.rcRoll = self.outputs[0]
        self.drone_cmd.rcPitch = self.outputs[1]
        self.drone_cmd.rcYaw = 1500.0
        self.drone_cmd.rcThrottle = 0 if self.outputs[2] == 0 else self.outputs[2]

        self.x_error_pub.publish(errors[0])
        self.y_error_pub.publish(errors[1])
        self.z_error_pub.publish(errors[2])
        self.zero_error_pub.publish(0.0)
        self.drone_pub.publish(self.drone_cmd)

    def pid(self):
        errors = [0.0, 0.0, 0.0]
        self.calculate_errors(errors)
        self.calculate_pid_eq(errors)
        self.set_bounds()
        self.set_prev_vals(errors)
        self.publish_values(errors)

if __name__ == '__main__':
    SYSTEM = ControlSystem()
    RATE = rospy.Rate(1 / SYSTEM.sample_time)
    while not rospy.is_shutdown():
        SYSTEM.pid()
        RATE.sleep()

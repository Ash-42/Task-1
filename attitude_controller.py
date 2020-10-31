#!/usr/bin/env python2

import time
import tf
import rospy
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class Edrone():

    def __init__(self):

        rospy.init_node('attitude_controller')

        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        self.setpoint_cmd = [0.0, 0.0, 0.0, 0.0]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        self.Kp = [0, 0, 0]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 0]

        self.prev_errors = [0.0, 0.0, 0.0]
        self.iTerm = [0.0, 0.0, 0.0]
        self.outputs = [0.0, 0.0, 0.0]
        self.thrust = 0.0
        self.min_value = 0
        self.max_value = 1023
        self.sample_time = 0.05

        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_error_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)
        self.zero_error_pub = rospy.Publisher('/zero_error', Float32, queue_size=1)

        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)

    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_cmd[3] = msg.rcThrottle

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.5
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.05

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.5
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.05

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 0.5
        self.Ki[2] = yaw.Ki * 0.008
        self.Kd[2] = yaw.Kd * 0.05

    def transform_inputs(self):
        (self.drone_orientation_euler[1],
         self.drone_orientation_euler[0],
         self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
        [self.drone_orientation_quaternion[0],
         self.drone_orientation_quaternion[1],
         self.drone_orientation_quaternion[2],
         self.drone_orientation_quaternion[3]])

        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
        self.thrust = self.setpoint_cmd[3] * 1.023 - 1023

    def calculate_errors(self, errors):
        errors[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
        errors[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
        errors[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]

    def calculate_pid_eq(self, errors):
        pTerm = [self.Kp[0] * errors[0],
                 self.Kp[1] * errors[1],
                 self.Kp[2] * errors[2]]

        self.iTerm[0] = self.Ki[0] * (self.iTerm[0] + errors[0])
        self.iTerm[1] = self.Ki[1] * (self.iTerm[1] + errors[1])
        self.iTerm[2] = self.Ki[2] * (self.iTerm[2] + errors[2])

        dTerm = [self.Kd[0] * (errors[0] - self.prev_errors[0]),
                 self.Kd[1] * (errors[1] - self.prev_errors[1]),
                 self.Kd[2] * (errors[2] - self.prev_errors[2])]

        self.outputs[0] = pTerm[0] + self.iTerm[0] + dTerm[0]
        self.outputs[1] = pTerm[1] + self.iTerm[1] + dTerm[1]
        self.outputs[2] = pTerm[2] + self.iTerm[2] + dTerm[2]

    def set_prop_bounds(self, prop_speed):
        if prop_speed > self.max_value:
            return self.max_value
        if prop_speed < self.min_value:
            return self.min_value
        return prop_speed

    def calculate_prop_speeds(self):
        out_roll, out_pitch, out_yaw = [i for i in self.outputs]
        self.pwm_cmd.prop1 = self.thrust + out_roll - out_pitch + out_yaw   # front right
        self.pwm_cmd.prop2 = self.thrust + out_roll + out_pitch - out_yaw   # back right
        self.pwm_cmd.prop3 = self.thrust - out_roll + out_pitch + out_yaw   # back left
        self.pwm_cmd.prop4 = self.thrust - out_roll - out_pitch - out_yaw   # front left

        pwm_cmds = [self.pwm_cmd.prop1,
                    self.pwm_cmd.prop2,
                    self.pwm_cmd.prop3,
                    self.pwm_cmd.prop4]
        [self.pwm_cmd.prop1,
         self.pwm_cmd.prop2,
         self.pwm_cmd.prop3,
         self.pwm_cmd.prop4] = pwm_cmds = map(self.set_prop_bounds, pwm_cmds)

    def set_prev_vals(self, errors):
        self.prev_errors[0] = errors[0]
        self.prev_errors[1] = errors[1]
        self.prev_errors[2] = errors[2]

    def publish_values(self, errors):
        self.roll_error_pub.publish(errors[0])
        self.pitch_error_pub.publish(errors[1])
        self.yaw_error_pub.publish(errors[2])
        self.zero_error_pub.publish(0.0)
        self.pwm_pub.publish(self.pwm_cmd)

    def pid(self):
        errors = [0.0, 0.0, 0.0]
        self.transform_inputs()
        self.calculate_errors(errors)
        self.calculate_pid_eq(errors)
        self.calculate_prop_speeds()
        self.set_prev_vals(errors)
        self.publish_values(errors)
        print 'Thrust: {}'.format(self.thrust)

if __name__ == '__main__':
    E_DRONE = Edrone()
    RATE = rospy.Rate(1 / E_DRONE.sample_time)
    while not rospy.is_shutdown():
        E_DRONE.pid()
        RATE.sleep()

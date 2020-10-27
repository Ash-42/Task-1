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

        self.setpoint_cmd = [0.0, 0.0, 0.0]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        self.Kp = [0, 0, 0]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 0]

        self.prev_values = [0.0, 0.0, 0.0]
        self.iTerm = [0.0, 0.0, 0.0]
        self.outputs = [0.0, 0.0, 0.0]

        self.min_values = [0, 0, 0, 0]
        self.max_values = [1024, 1024, 1024, 1024]

        self.last_time = 0.0
        self.sample_time = 0.060

        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float32, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)
        self.yaw_error_pub = rospy.Publisher('/yaw_error', Float32, queue_size=1)

        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.alt_set_pid)

    def imu_callback(self, msg):
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    def drone_command_callback(self, msg):
        print 'Commands : [{}, {}, {}]'.format(msg.rcRoll, msg.rcPitch, msg.rcYaw)
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp
        self.Ki[0] = roll.Ki * self.sample_time
        self.Kd[0] = roll.Kd / self.sample_time

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp
        self.Ki[1] = pitch.Ki * self.sample_time
        self.Kd[1] = pitch.Kd / self.sample_time

    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp
        self.Ki[2] = yaw.Ki * self.sample_time
        self.Kd[2] = yaw.Kd / self.sample_time

    def alt_set_pid(self, alt):
        self.throttle = alt

    def transform_inputs(self):
        (self.drone_orientation_euler[0],
         self.drone_orientation_euler[1],
         self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion(
        [self.drone_orientation_quaternion[0],
         self.drone_orientation_quaternion[1],
         self.drone_orientation_quaternion[2],
         self.drone_orientation_quaternion[3]])

        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30

    def calculate_errors(self, errors):
        errors[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
        errors[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
        errors[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]

        self.roll_error_pub.publish(errors[0])
        self.pitch_error_pub.publish(errors[1])
        self.yaw_error_pub.publish(errors[2])

    def set_value_bounds(self, values):
        if values == 'inputs':
            for i in range(len(self.iTerm)):
                if self.iTerm[i] > self.max_values[i]:
                    self.iTerm[i] = self.max_values[i]
                if self.iTerm[i] < self.min_values[i]:
                    self.iTerm[i] = self.min_values[i]
        if values == 'outputs':
            for i in range(len(self.outputs)):
                if self.outputs[i] > self.max_values[i]:
                    self.outputs[i] = self.max_values[i]
                if self.outputs[i] < self.min_values[i]:
                    self.outputs[i] = self.min_values[i]

    def calculate_pwm_eq(self, errors):
        self.iTerm[0] += (self.Ki[0] * errors[0])
        self.iTerm[1] += (self.Ki[1] * errors[1])
        self.iTerm[2] += (self.Ki[2] * errors[2])

        self.set_value_bounds(values='inputs')

        delta_roll = self.drone_orientation_euler[0] - self.prev_values[0]
        delta_pitch = self.drone_orientation_euler[1] - self.prev_values[1]
        delta_yaw = self.drone_orientation_euler[2] - self.prev_values[2]

        self.outputs[0] = self.Kp[0] * errors[0] + self.iTerm[0] - self.Kd[0] * delta_roll
        self.outputs[1] = self.Kp[1] * errors[1] + self.iTerm[1] - self.Kd[1] * delta_pitch
        self.outputs[2] = self.Kp[2] * errors[2] + self.iTerm[2] - self.Kd[2] * delta_yaw

        self.set_value_bounds(values='outputs')

    def set_prev_vals(self):
        self.prev_values[0] = self.drone_orientation_euler[0]
        self.prev_values[1] = self.drone_orientation_euler[1]
        self.prev_values[2] = self.drone_orientation_euler[2]
        self.last_time = now

    def pid(self):
        now = time.time()
        delta_time = now - self.last_time
        if delta_time >= self.sample_time:

            errors = [0.0, 0.0, 0.0]
            self.transform_inputs()
            self.calculate_errors(errors)
            print 'Errors: [{}, {}, {}]'.format(errors[0], errors[1], errors[2])
            self.calculate_pid_eq(errors)
            print 'PWM Outputs: [{}, {}, {}]'.format(self.outputs[0], self.outputs[1], self.outputs[2])
            self.set_prev_vals()
            self.pwm_pub.publish(self.pwm_cmd)

if __name__ == '__main__':
    E_DRONE = Edrone()
    RATE = rospy.Rate(1 / E_DRONE.sample_time)
    while not rospy.is_shutdown():
        E_DRONE.pid()
        RATE.sleep()

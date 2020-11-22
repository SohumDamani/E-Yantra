#!/usr/bin/env python



# '''
# This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
# This node publishes and subsribes the following topics:
#         PUBLICATIONS            SUBSCRIPTIONS
#         /roll_error             /pid_tuning_altitude
#         /pitch_error            /pid_tuning_pitch
#         /yaw_error              /pid_tuning_roll
#         /edrone/pwm             /edrone/imu/data
#                                 /edrone/drone_command

# Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
# CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
# '''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

		# This corresponds to your current orientation of eDrone in quaternion format. This value must be 
		#updated each time in your imu callback
		# [x,y,z,w]
		self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

		# This corresponds to your current orientation of eDrone converted in euler angles form.
		# [r,p,y]
		self.drone_orientation_euler = [0.0, 0.0, 0.0]

		# This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
		# [r_setpoint, p_setpoint, y_setpoint]
		self.setpoint_cmd = [0.0, 0.0, 0.0]

		# The setpoint of orientation in euler angles at which you want to stabilize the drone
		# [r_setpoint, p_psetpoint, y_setpoint]
		self.setpoint_euler = [0.0, 0.0, 0.0]

		# Declaring pwm_cmd of message type prop_speed and initializing values
		# Hint: To see the message structure of prop_speed type the following command in the terminal
		# rosmsg show vitarana_drone/prop_speed

		self.pwm_cmd = prop_speed()
		self.pwm_cmd.prop1 = 0.0
		self.pwm_cmd.prop2 = 0.0
		self.pwm_cmd.prop3 = 0.0
		self.pwm_cmd.prop4 = 0.0

		# initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
		# after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [232.4, 252.4, 1480]
		self.Ki = [0, 0, 0]
		self.Kd = [660, 670, 0]
		# -----------------------Add other required variables for pid here ----------------------------------------------
		self.throttle=0 # to store throttle speed input

		# to publish pitch,roll and yaw error respectively
		self.pitch_error=Float32()
		self.roll_error=Float32()
		self.yaw_error=Float32()        

		#to store the data for errors
		self.pitch_error.data=0.0
		self.roll_error.data=0.0
		self.yaw_error.data=0.0

		#to store pid error
		self.out_roll = 0
		self.out_pitch = 0
		self.out_yaw = 0

		self.error = [0,0,0] # to store currnt error in roll,pitch,yaw
		self.prev_error=[0,0,0] # to store previous error in roll,pitch
		self.Iterm=[0,0,0] # for calculating intergral error of pid
		self.min_values=[0,0,0,0]
		self.max_values=[1024,1024,1024,1024]

		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
		#        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
		#                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
		#
		# ----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 25

		# Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
		self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

		self.yaw_er_pub=rospy.Publisher('/yaw_error',Float32,queue_size=1)
		self.pitch_er_pub=rospy.Publisher('/pitch_error',Float32,queue_size=1)
		self.roll_er_pub=rospy.Publisher('/roll_error',Float32,queue_size=1)
		# ------------------------Add other ROS Publishers here-----------------------------------------------------

		# -----------------------------------------------------------------------------------------------------------

		# Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
		rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
		rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
		#rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		#rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		#rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)

	def imu_callback(self, msg):

		self.drone_orientation_quaternion[0] = msg.orientation.x
		self.drone_orientation_quaternion[1] = msg.orientation.y
		self.drone_orientation_quaternion[2] = msg.orientation.z
		self.drone_orientation_quaternion[3] = msg.orientation.w


	def drone_command_callback(self, msg):
		self.setpoint_cmd[0] = msg.rcRoll
		self.setpoint_cmd[1] = msg.rcPitch
		self.setpoint_cmd[2] = msg.rcYaw
		self.throttle = msg.rcThrottle
		self.throttle = (self.throttle - 1000 ) * 1.024 #converting throttle value from 1000-2000 to 0-1024
		# ---------------------------------------------------------------------------------------------------------------

	def roll_set_pid(self, roll):
		self.Kp[0] = roll.Kp *2.9 
		self.Ki[0] = roll.Ki *0.3
		self.Kd[0] = roll.Kd *2.2

	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.Kp *5
		self.Ki[1] = pitch.Ki *0.3
		self.Kd[1] = pitch.Kd *0.5

	def yaw_set_pid(self, yaw):
		self.Kp[2] = yaw.Kp * 3.15
		self.Ki[2] = yaw.Ki * 0.3
		self.Kd[2] = yaw.Kd * 1.9 

	# ----------------------------------------------------------------------------------------------------------------------

	def pid(self):

		# Converting quaternion to euler angles
		(self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

		# Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
		self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
		self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
		self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30


		# Computing errors
		self.error[0]=self.setpoint_euler[0] - self.drone_orientation_euler[0]
		self.error[1]=self.setpoint_euler[1] - self.drone_orientation_euler[1]
		self.error[2]=self.setpoint_euler[2] - self.drone_orientation_euler[2]

		self.Iterm[0]=(self.Iterm[0]+self.error[0])*self.Ki[0]
		self.Iterm[1]=(self.Iterm[1]+self.error[1])*self.Ki[1]
		self.Iterm[2]=(self.Iterm[2]+self.error[2])*self.Ki[2]

		self.out_roll= self.Kp[0]*self.error[0] + self.Iterm[0] + (self.error[0] - self.prev_error[0])*self.Kd[0]
		self.out_pitch=self.Kp[1]*self.error[1] + self.Iterm[1] + (self.error[1]-self.prev_error[1])*self.Kd[1]
		self.out_yaw=self.Kp[2]*self.error[2] +   self.Iterm[2] + (self.error[2]-self.prev_error[2])*self.Kd[2] 

		self.prev_error[0]=self.error[0]
		self.prev_error[1]=self.error[1]
		self.prev_error[2]=self.error[2]

		# setting propeller speed acooring to pid error
		self.pwm_cmd.prop1  = self.throttle - self.out_pitch + self.out_roll - self.out_yaw
		self.pwm_cmd.prop2 = self.throttle  - self.out_pitch - self.out_roll + self.out_yaw
		self.pwm_cmd.prop3  = self.throttle + self.out_pitch - self.out_roll - self.out_yaw
		self.pwm_cmd.prop4 = self.throttle  + self.out_pitch + self.out_roll + self.out_yaw


		# limiting the max and min speed of propellers

		if self.pwm_cmd.prop1 > self.max_values[0]:
			self.pwm_cmd.prop1 = self.max_values[0]
		elif self.pwm_cmd.prop1 < self.min_values[0]:
			self.pwm_cmd.prop1 = self.min_values[0]
		
		if self.pwm_cmd.prop2 > self.max_values[1]:
			self.pwm_cmd.prop2 = self.max_values[1]
		elif self.pwm_cmd.prop2 < self.min_values[1]:
			self.pwm_cmd.prop2 = self.min_values[1]
		
		if self.pwm_cmd.prop3 > self.max_values[2]:
			self.pwm_cmd.prop3 = self.max_values[2]
		elif self.pwm_cmd.prop3 < self.min_values[2]:
			self.pwm_cmd.prop3 = self.min_values[2]
		
		if self.pwm_cmd.prop4 > self.max_values[3]:
			self.pwm_cmd.prop4 = self.max_values[3]
		elif self.pwm_cmd.prop4 < self.min_values[3]:
			self.pwm_cmd.prop4 = self.min_values[3]

		self.pwm_pub.publish(self.pwm_cmd)
		self.yaw_er_pub.publish(self.yaw_error)
		self.pitch_er_pub.publish(self.pitch_error)
		self.roll_er_pub.publish(self.roll_error)

if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()

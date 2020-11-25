#!/usr/bin/env python

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix,LaserScan
from std_msgs.msg import Float32
import rospy
import time
import tf
import rosservice
from std_msgs.msg import String
from vitarana_drone.srv import *
import math
from numpy import inf

class Edrone():
	def __init__(self):
		rospy.init_node("position_controller")

		self.set_loc = [19.0007046575,71.9998955286,25.000000]      
		self.final_loc= [0.0,0.0,0.0]
		self.current_loc=[0.0,0.0,0.0]

		self.lat_error = 0.0
		self.long_error =0.0 
		self.alt_error= 0.0

		self.out_throttle=0.0
		self.out_pitch=0.0
		self.out_roll=0.0
		self.prev_error=[0.0,0.0,0.0]
		
		self.final_cmd = edrone_cmd() 
		self.final_cmd.rcRoll = 0.0
		self.final_cmd.rcPitch = 0.0
		self.final_cmd.rcYaw = 0.0
		self.final_cmd.rcThrottle = 0.0

		# self.alt_kp=41.27
		# self.alt_ki=0.0
		# self.alt_kd=3500
		self.alt_kp=30
		self.alt_ki=0.0
		self.alt_kd=3700

		self.lat_kp=26000
		self.lat_kd=8300000
		self.lat_ki=0.0

		self.long_kp=28000
		self.long_kd=9400000
		self.long_ki=0.0

		self.Iterm=[0.0,0.0,0.0]
		self.flag=1

		self.out_lat=0.0

		self.out_long=0.0
		self.out_alt=0.0

		self.is_pickable='False'

		self.sample_time = 40
		self.left_margin=0.0
		self.back_margin=0.0
		self.obs_front=0.0
		self.obs_back=0.0
		self.obs_right=0.0
		self.obs_left=0.0
		self.flag_left=1
		self.flag_pick=1
		self.flag_front_done=1
		self.flag_left_done=1
		self.attitude_input_pub = rospy.Publisher('/drone_command', edrone_cmd,queue_size=1) 

		
		
		
		rospy.Subscriber('/edrone/gps', NavSatFix, self.loc_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune, self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid)
		rospy.Subscriber('/target',String, self.set_location) 
		rospy.Subscriber('/edrone/gripper_check', String, self.check_if_available)  

		rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.set_obs_distances)     	

	def check_if_available(self,ans):
		self.is_pickable=ans.data
	
	def set_obs_distances(self,message):
		
		self.obs_front=message.ranges[4]
		self.obs_back=message.ranges[2]
		self.obs_right=message.ranges[1]
		self.obs_left=message.ranges[3]


	def loc_callback(self,msg):
		self.current_loc[0] = msg.latitude
		self.current_loc[1] = msg.longitude
		self.current_loc[2] = msg.altitude
	
	def set_location(self,tar):
		loc = tar.data
		loc = loc.replace("'","")
		loc = loc.replace("data:","")
		seperate = loc.split(",")
		self.final_loc[0] = float(seperate[0])
		self.final_loc[1] = float(seperate[1])
		self.final_loc[2] = float(seperate[2])
		self.flag=0
		print self.final_loc

	def altitude_set_pid(self, altitude):

		self.alt_kp = altitude.Kp *0.01
		self.alt_ki = altitude.Ki *0.7
		self.alt_kd = altitude.Kd *0.9

	def latitude_set_pid(self, pitch):
		self.lat_kp = pitch.Kp *100
		self.lat_ki = pitch.Ki *1
		self.lat_kd = pitch.Kd *1500


	def longitude_set_pid(self, roll):
		self.long_kp = roll.Kp *100
		self.long_ki = roll.Ki *1
		self.long_kd = roll.Kd *1500

	def pid_p(self): 

		self.alt_error=self.set_loc[2] - self.current_loc[2]

 		
		self.out_alt= self.alt_kp*self.alt_error + self.Iterm[2] + (self.alt_error - self.prev_error[2])*self.alt_kd

		self.prev_error[2]=self.alt_error
		self.Iterm[2]=(self.Iterm[2]+self.alt_error)*self.alt_ki


		if(self.current_loc[2]>=25.00 or abs(self.current_loc[0]-self.set_loc[0])<=0.000001):
			self.lat_error=self.set_loc[0] - self.current_loc[0] - self.left_margin

			self.out_lat=self.lat_kp*self.lat_error + self.Iterm[0] + (self.lat_error - self.prev_error[0])*self.lat_kd
			self.prev_error[0]=self.lat_error
			self.Iterm[0] =(self.Iterm[0]+self.lat_error)*self.lat_ki

		if(self.current_loc[2]>=25.00 or abs(self.current_loc[1]-self.set_loc[1])<=0.000001):
			self.long_error=self.set_loc[1] - self.current_loc[1] + self.back_margin

			self.out_long=self.long_kp*self.long_error + self.Iterm[1] + (self.long_error - self.prev_error[1])*self.long_kd
			self.prev_error[1]=self.long_error
			self.Iterm[1] =(self.Iterm[1]+self.long_error)*self.long_ki



		if(self.current_loc[0]>=19.0007043000 and self.current_loc[0]<=19.0007047000 and self.flag==1):
			self.set_loc[2]=22
			self.set_loc[1]=71.9998955286
			self.set_loc[0]=19.0007046575




		if(self.is_pickable=='True' and self.flag_pick==1):
			call_service=rospy.ServiceProxy('/edrone/activate_gripper',Gripper)
			call_service(True)

			self.set_loc[0]=self.final_loc[0]
			self.set_loc[1]=self.final_loc[1]
			self.set_loc[2]=26
			self.lat_kp=16000
			self.long_kp=18000
			self.flag_pick=0

		
		if(self.obs_left<=18.0000000 and self.obs_left>=5.0000000 and self.obs_left is not inf and self.obs_left!=0 and self.flag_left_done==1):
			print('I m in')
			print(self.obs_left)
			self.set_loc[0]=self.current_loc[0] #- 0.03*(self.obs_left - 8)*0.000009034
			self.left_margin=(self.obs_left - 8)*0.000009034
			self.set_loc[1]=self.current_loc[1] 

			self.back_margin=1.5*0.000009497


			if(self.flag_left==1):

				self.out_alt=0.0
				self.out_lat=0.0
				self.out_lang=0.0

				self.prev_error=[0.0,0.0,0.0]
				self.lat_error=0.0
				self.long_error=0.0
				self.flag_left=0
		

		if(self.obs_front<=8.000 and self.obs_front>=2.0000000 and (self.obs_front is not inf) and self.obs_front!=0 and self.flag_front_done==1):
			self.back_margin=-1.5*(self.obs_front - 6)*0.000009497

		print(self.obs_front)

		if((self.obs_left==inf) and self.set_loc[0]!=19.0007046575 and self.set_loc[1]!=71.9998955286 and self.set_loc[2]!=22):
			self.set_loc[0]=self.final_loc[0]
			self.set_loc[1]=self.final_loc[1]
			self.set_loc[2]=25
			# self.back_margin=0.0
			self.left_margin=0.0
			self.flag_left_done=0
			self.lat_kp=14000
			self.long_kp=16000
	


		if(self.current_loc[0]>=(self.final_loc[0]-0.0001) and self.current_loc[0]<=(self.final_loc[0]+0.0001)):
			self.back_margin=0.0
			self.lat_kp=26000
			self.long_kp=28000
			
		if(self.current_loc[0]>=(self.final_loc[0]-0.00001) and self.current_loc[0]<=(self.final_loc[0]+0.00001)):
			self.set_loc[2]=6
				


		self.out_throttle= 1500 + self.out_alt

		
		self.out_pitch=1500 + self.out_lat

		self.out_roll=1500 + self.out_long


		self.final_cmd.rcRoll= self.out_roll
		self.final_cmd.rcYaw = 1500.0
		self.final_cmd.rcPitch = self.out_pitch
		self.final_cmd.rcThrottle = self.out_throttle


		

		self.attitude_input_pub.publish(self.final_cmd)


if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid_p()
		r.sleep()
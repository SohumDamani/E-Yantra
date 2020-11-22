#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Float32
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode

class image_proc():

	# Initialise everything
	def __init__(self):

		self.lat =Float32()
		self.lon =Float32()
		self.alt =Float32()

		self.lat.data = 0
		self.lon.data = 0
		self.alt.data = 0

		rospy.init_node('barcode_test') #Initialise rosnode 
		self.set_latitude = rospy.Publisher("/target_latitude",Float32,queue_size=1)#publish the target point
		self.set_longitude = rospy.Publisher("/target_longitude",Float32,queue_size=1)
		self.set_altitude = rospy.Publisher("/target_altitude",Float32,queue_size=1)
		
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()


	# Callback function of amera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return
	def qrcode(self):
		barcodes = decode(self.img)
		for barcode in barcodes:
			barcodeData = barcode.data.decode("utf-8")
			uts = barcodeData.encode('ascii', 'ignore')
			loc = uts.split(',')
			self.lat.data = float(loc[0])
			self.lon.data = float(loc[1])
			self.alt.data = float(loc[2])

			self.set_latitude.publish(self.lat)
			self.set_longitude.publish(self.lon)
			self.set_altitude.publish(self.alt)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
    while True:
    	image_proc_obj.qrcode()
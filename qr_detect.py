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

		self.target=String()
		self.target.data = ""

		rospy.init_node('barcode_test') #Initialise rosnode 	
		self.target_pub = rospy.Publisher("/target",String,queue_size=1)
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.bridge = CvBridge()


	# Callback function of amera topic
	def image_callback(self, data):
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return
		barcodes=[]
		barcodes = decode(img)
		for barcode in barcodes:
			barcodeData = barcode.data.decode("utf-8")
			self.target.data = barcodeData

			self.target_pub.publish(self.target)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()

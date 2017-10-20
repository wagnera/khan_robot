#!/usr/bin/env python

import RPi.GPIO as GPIO
import quadrature_v2
from sensor_msgs.msg import JointState
import rospy
import roslib

class encoder_reader:
	def __init__(self):
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(29, GPIO.IN)#Left Encoder State A
		GPIO.setup(31, GPIO.IN)#Left Encoder State B
		GPIO.setup(33, GPIO.IN)#Right Encoder State A
		GPIO.setup(35, GPIO.IN)#Right Encoder State B
		
		self._encoder_leftA_state = None
		self._encoder_leftB_state = None
		self._encoder_rightA_state = None
		self._encoder_rightB_state = None
		
	def read(self):
		self._encoder_leftA_state = GPIO.input(29)
		self._encoder_leftB_state = GPIO.input(31)
		self._encoder_rightA_state = GPIO.input(33)
		self._encoder_rightB_state = GPIO.input(35)
		
	#def publish(data):
	@property
  	def encoder_leftA_state(self):
  		return self._encoder_leftA_state
  	@property
  	def encoder_leftB_state(self):
  		return self._encoder_leftB_state
  	@property
  	def encoder_rightA_state(self):
  		return self._encoder_rightA_state
  	@property
  	def encoder_rightB_state(self):
  		return self._encoder_rightB_state
if __name__ == '__main__':
	QuadUpdate = encoder_reader()#creating instance of encoder_reader class 
	QuadEstimate_Left = quadrature_v2.QuadratureEstimator()
	QuadEstimate_Right = quadrature_v2.QuadratureEstimator()

	try:
		#Initialize node
		rospy.init_node('encoder_read')
		print("printng encoder_read")
		rate = rospy.Rate(2000)
		print("setrate")
		FLW_pub = rospy.Publisher('feedback', JointState, queue_size=10)
		#FRW_pub = rospy.Publisher('front_right_wheel/encoder', JointState, queue_size=10)
		print("publishers set")
		FR = JointState()
		FL = JointState()
		RR = JointState()
		RL = JointState()
		print("messages created")
		while not rospy.is_shutdown():
			QuadUpdate.read() #calling update to pins
			QuadEstimate_Left.update(QuadUpdate.encoder_leftA_state,QuadUpdate.encoder_leftB_state,rospy.get_time())
			QuadEstimate_Right.update(QuadUpdate.encoder_rightA_state,QuadUpdate.encoder_rightB_state,rospy.get_time())
			FL.name=["l","r"]
			#FR.name=
			FL.velocity = [QuadEstimate_Left.velocity,QuadEstimate_Right.velocity]#add velocity data to message
			#FR.velocity[1] = [QuadEstimate_Right.velocity]
			FL.position = [QuadEstimate_Left.position,QuadEstimate_Right.position]#add position data to message
			#FR.position[1] = [QuadEstimate_Right.position]
			try:
				FLW_pub.publish(FL) #publish msg to joint_state
				#FLW_pub.publish(FR) 
			except rospy.ROSInterruptException:
				pass
			rate.sleep()
	except:
		pass
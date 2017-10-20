#!/usr/bin/env python

#This script subscribes to the topics fr each motor command and using this input calculates 
#the required PWM, GPIO pins and settings to acheive the required motion 
import RPi.GPIO as GPIO
import RPi
from sensor_msgs.msg import JointState
import rospy
import roslib

class locomotion:
	def __init__(self): #sets up GPIO pins as outputs
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(11, GPIO.OUT) #Left motor dir
		GPIO.setup(12, GPIO.OUT) #Right motor dir
		GPIO.setup(13, GPIO.OUT) #Left motor PWM
		GPIO.setup(15, GPIO.OUT) #Right motor PWM
		GPIO.setup(36, GPIO.OUT) #Left back motor dir
		GPIO.setup(38, GPIO.OUT) #Right back motor dir
		GPIO.setup(37, GPIO.OUT) #Left back motor PWM
		GPIO.setup(40, GPIO.OUT) #Right back motor PWM
		self.left=GPIO.PWM(13,2000)
		self.right=GPIO.PWM(15,2000)
		self.leftb=GPIO.PWM(37,2000)
		self.rightb=GPIO.PWM(40,2000)
		self.left.start(0)
		self.right.start(0)
		self.leftb.start(0)
		self.rightb.start(0)
		print("PWM start")        

	#def Rmove(self, motor, speed):
	def Rmove(self, data):
		speedl = 5*data.velocity[0]
		speedr = 5*data.velocity[1]
		
		if speedl > 100:
			speedl = 100
		if speedr > 100:
			speedl = 100

		speedPWM = abs(int(speedl))#convert input velocity to a pwm value
		if speedPWM > 100:
			speedPWM=100
		self.left.ChangeDutyCycle(speedPWM)#start pwm for that motor
		self.leftb.ChangeDutyCycle(speedPWM)
		if speedl > 0: #if the direction is forward
			GPIO.output(11, True)
			GPIO.output(36, False)
		elif speedl < 0: #if the direction is backwards
			GPIO.output(11, False)
			GPIO.output(36, True)
		else:
			pass#locomotion.PinReset(self,1) #reset the pins
		
		speedPWM = abs(int(speedr))
		if speedPWM > 100:
			speedPWM=100
		self.right.ChangeDutyCycle(speedPWM)
		self.rightb.ChangeDutyCycle(speedPWM)
		if speedr > 0:
			GPIO.output(12, True)
			GPIO.output(38, False)
		elif speedr < 0:
			GPIO.output(12, False)
			GPIO.output(38, True)
		else:
			pass#locomotion.PinReset(self,2)
	def exit_hook(self):
		GPIO.cleanup()

# If this is loaded as the main python file, execute the main details
if __name__ == '__main__':
  Robert=locomotion()
  rospy.on_shutdown(Robert.exit_hook)
  #Robert.__init__()
  try:
    #Initialize node
    rospy.init_node('motor_control')
       
    #create subscriber that subscribes to the /ticks topic and uses the callback function
    W_sub = rospy.Subscriber('motor_cmd', JointState, Robert.Rmove)
    
    #We need to wait for new messages
    rospy.spin()
  #If we are interrupted, catch the exception, but do nothing
  except rospy.ROSInterruptException:
    exit_hook()



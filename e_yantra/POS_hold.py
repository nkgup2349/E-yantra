#!/usr/bin/env python3
'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					


								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from swift_msgs.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from sensor_msgs import *
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from luminosity_drone.msg import Biolocation
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
import time



class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [5,7,20] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [30.44, 45, 30]
		self.Ki = [0.01, 0.01, 0.07]
		self.Kd = [167.2, 285, 365.5]

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.error = [0,0,0]    #variables for calculating PID Errors
		self.prev_error = [0,0,0]
		self.differential_error = [0,0,0]
		self.sum_error = [0,0,0]

		self.min_values = [1000,1000,1000]
		self.max_values = [2000,2000,2000]
        
        # Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.033 # in seconds





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.throttle_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.biolocation_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=1)
	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/image_raw', Image, self.image_callback)
		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE
		self.bridge = CvBridge()  # Create a bridge to convert ROS Image to OpenCV Image


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)
	def image_callback(self, msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert ROS Image to OpenCV Image
			organism_type, whycon_x, whycon_y, whycon_z = self.detect_organism(cv_image)
			if organism_type is not None:
				biolocation_msg = Biolocation()
				biolocation_msg.organism_type = organism_type
				biolocation_msg.whycon_x = whycon_x
				biolocation_msg.whycon_y = whycon_y
				biolocation_msg.whycon_z = whycon_z
				self.biolocation_pub.publish(biolocation_msg)
		except Exception as e:
				rospy.logerr("Error processing image: {}".format(str(e)))
	def detect_organism(self, cv_image):
    # Convert the image to grayscale
		gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Apply a threshold to segment the image (adjust parameters as needed)
		_, binary_image = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY)

    # Find contours in the binary image
		contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterate through the contours to identify organisms
		for contour in contours:
        # Calculate the centroid of the contour
			M = cv2.moments(contour)
			if M["m00"] != 0:
				cx = int(M["m10"] / M["m00"])
				cy = int(M["m01"] / M["m00"])

            # Draw a circle at the centroid for visualization (optional)
				cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)

            # Determine organism type based on the number of LEDs (example values, adapt as needed)
			if len(contour) == 2:
				organism_type = "alien_a"
			elif len(contour) == 3:
				organism_type = "alien_b"
			elif len(contour) == 4:
				organism_type = "alien_c"
			else:
				organism_type = "unknown"

            # Return the organism type and its centroid coordinates
			return organism_type, cx, cy, 0.0  # Assuming z-coordinate is 0.0 for simplicity

    # If no organisms are detected
		return None, 0.0, 0.0, 0.0
	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06# This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.0008
		self.Kd[2] = alt.Kd * 0.3
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def roll_set_pid(self,roll): #roll
		self.Kp[0] = roll.Kp * 0.06
		self.Ki[0] = roll.Ki * 0.0008
		self.Kd[0] = roll.Kd * 0.3

	def pitch_set_pid(self,pitch): #pitch
		self.Kp[1] = pitch.Kp * 0.006
		self.Ki[1] = pitch.Ki * 0.0008
		self.Kd[1] = pitch.Kd * 0.3

	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

		self.error[0] = -(self.drone_position[0] - self.setpoint[0])
		self.error[1] = (self.drone_position[1] - self.setpoint[1])
		self.error[2] = (self.drone_position[2] - self.setpoint[2])

		self.differential_error[0] = self.error[0] - self.prev_error[0]
		self.differential_error[1] = self.error[1] - self.prev_error[1]
		self.differential_error[2] = self.error[2] - self.prev_error[2]

		self.sum_error[0] = self.sum_error[0] + self.error[0]
		self.sum_error[1] = self.sum_error[1] + self.error[1]
		self.sum_error[2] = self.sum_error[2] + self.error[2]

		self.cmd.rcRoll = int(1500 + self.error[0]*self.Kp[0] + self.differential_error[0]*self.Kd[0]+ self.sum_error[0]*self.Ki[0])
		self.cmd.rcPitch = int(1500 + self.error[1]*self.Kp[1] + self.differential_error[1]*self.Kd[1] + self.sum_error[1]*self.Ki[1])
		self.cmd.rcThrottle = int(1560 + self.error[2]*self.Kp[2] + self.differential_error[2]*self.Kd[2] + self.sum_error[2]*self.Ki[2])

		if self.cmd.rcThrottle > self.max_values[2]:
			self.cmd.rcThrottle = self.max_values[2]
		if self.cmd.rcThrottle < self.min_values[2]:
			self.cmd.rcThrottle = self.min_values[2]		

		if self.cmd.rcPitch > self.max_values[1]:
			self.cmd.rcPitch = self.max_values[1]	
		if self.cmd.rcPitch < self.min_values[1]:
			self.cmd.rcPitch = self.min_values[1]

		if self.cmd.rcRoll > self.max_values[0]:
			self.cmd.rcRoll = self.max_values[0]
		if self.cmd.rcRoll < self.min_values[0]:
			self.cmd.rcRoll = self.min_values[0]

	#------------------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)
		
		self.prev_error[0]=self.error[0]
		self.prev_error[1]=self.error[1]
		self.prev_error[2]=self.error[2]

		
		self.roll_error_pub.publish(self.error[0])
		self.pitch_error_pub.publish(self.error[1])
		self.throttle_error_pub.publish(self.error[2])



if __name__ == '__main__':

	swift_drone = swift()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		swift_drone.pid()
		r.sleep()

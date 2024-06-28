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

'''
# Team ID:          1168
# Theme:            Luminosity drone
# Author List:      Nikhil Kumar Gupta, Debadrita Nath, Asmita Chakroborty, Mohammad Amanullah
# Filename:         LD_1168_position_hold.py
# Functions:        __init__(self),__main__,,disarm(self),arm(self),whycon_callback(self,msg), altitude_set_pid(self,alt),roll_set_pid(self,roll), pitch_set_pid(self,pitch), pid(self)
'''




# Importing the required libraries

import rospy
from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from luminosity_drone.msg import Biolocation
from imutils import contours
from skimage import measure
from cv_bridge import  CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import imutils
import cv2
import time




class swift():
    """docstring for swift"""
    def __init__(self):
        
        rospy.init_node('drone_control')	# initializing ros node with name drone_control

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0,0.0,0.0]

        # [x_setpoint, y_setpoint, z_setpoint]
        # List of set points

        self.setpoint = [
            [-7, -7, 23], 
            [-7, 0, 23],                
            [-7, 7, 23],
            [-4, 7, 23],
            [-4, 0, 23],
            [-4 ,-7, 23],
            [-1, -7, 22],
            [-1, 0, 22],
            [-1, 7, 22],
            [2, 7, 22],
            [2, 3, 22],
            [2, 0, 22],
            [2, -3, 22],
            [2, -7, 22],
            [3.5, -7, 22],
            [3.5, -3, 22],
            [3.5, 0, 22],
            [3.5, 3, 22],
            [3.5, 7, 22],
            [6.5, 7, 22],
            [6.5, 3, 22],
            [6.5, 0, 22],
            [6.5, -3, 22],
            [6.5, -7, 22],
            [11, 11, 37]

        ]                       # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        
        # Initialize current set point index
        self.current_setpoint_index = 0      # Start at the takeoff position




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
        
        # Latest value

        self.Kp = [17.04, 24.9,35.4 ]
        self.Ki = [0, 0, 0]
        self.Kd = [268.5,321 , 321]

        
        #-----------------------Add other required variables for pid here ----------------------------------------------
        
        self.prev_error = [0,0,0]   # corresponding to [x, y, z]
        self.error=[0,0,0]          # corresponding to [x, y, z]
        self.sum_error = [0,0,0]    # corresponding to [x, y, z]

        #Max/min___values

        self.max_values = [2000,2000,2000]   # corresponding to [roll, pitch, throttle]
        self.min_values = [1000,1000,1000]   # corresponding to [roll, pitch, throttle]
   
         #--------------




        # Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        #													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]	You can change the upper limit and lower limit accordingly.    
        
       
         #----------------------------------------------------------------------------------------------------------

        # Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        self.sample_time = 0.0  #corresponding to [pitch, roll, throttle]	You can change the upper limit and lower limit accordingly.
   


        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)


        #------------------------Add other ROS Publishers here-----------------------------------------------------
    
        self.alt_error_pub=rospy.Publisher('/alt_error',Float64,queue_size=1)
        self.pitch_error_pub=rospy.Publisher('/pitch_error',Float64,queue_size=1)
        self.roll_error_pub=rospy.Publisher('/roll_error',Float64,queue_size=1)
        #self.biolocation_pub = rospy.Publisher('/astrobioloaction', Biolocation, queue_size = 1)


     
    #-----------------------------------------------------------------------------------------------------------


        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)


        #-------------------------Add other ROS Subscribers here----------------------------------------------------

        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber('/swift/camera_rgb/image_raw', Image, self.image_callback)

        
        self.bridge = CvBridge()
        
        
        #------------------------------------------------------------------------------------------------------------

        self.arm() # ARMING THE DRONE


    # Disarming condition of the drone
    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)


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
    # def altitude_set_pid(self,alt):

    def altitude_set_pid(self,alt):
        self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = alt.Ki * 0.0008
        self.Kd[2] = alt.Kd * 0.3

    #----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

    
    def roll_set_pid(self,roll):
        self.Kp[0] = roll.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[0] = roll.Ki * 0.0008
        self.Kd[0] = roll.Kd * 0.3

    
    def pitch_set_pid(self,pitch):
        self.Kp[1] = pitch.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[1] = pitch.Ki * 0.0008
        self.Kd[1] = pitch.Kd * 0.3

    
    # Adding this function to collect the image from the swift drone
    def image_callback(self, data):
        # print("Recieved image Message")
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            # print("Converted image Succesfully")
            num_leds, centroid_coordinates= self.detect_oraganism(self.image)
        except CvBridgeError as e:
            print(f"Imgage conversion error: {e}")



    



    



    #---------------------------------------------------------------------------------------------------------------


    #-----------------------------Write the PID algorithm here--------------------------------------------------------------
    
    # Steps:x...
    #       1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
    #	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
    #	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
    #	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
    #	5. Don't run the pid continously. Run the pid only
    # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in  at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
    #	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
    #																														self.cmd.rcPitch = self.max_values[1]
    #	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
    #	8. Add error_sum
    
    
    #-------------------------------------------------------------------------------------------------------------------------------------
    
    

    def pid(self):
         
 

        if self.current_setpoint_index < len(self.setpoint):
        
           #______along x axis

            self.error[0]= -(self.drone_position[0] - self.setpoint[self.current_setpoint_index][0])  
            self.cmd.rcRoll= int(1500 + (self.error[0]*self.Kp[0]) + ((self.error[0] - self.prev_error[0])*self.Kd[0])+(self.sum_error[0]*self.Ki[0]))
            self.sum_error[0] = self.sum_error[0] + self.error[0] 

            if self.cmd.rcRoll >self.max_values[0]:
                self.cmd.rcRoll = self.max_values[0]

            elif self.cmd.rcRoll <  self.min_values[0]:
                self.cmd.rcRoll =  self.min_values[0] 

            # Check if the drone is within the error range in all coordinate axes

            # if all(abs(self.error[0] <= 0.2)):
            # 	self.current_setpoint_index += 1     # Move to the next setpoint


        #------------------------------------------------------------------------------------------------------------

        #______along y axis 

            self.error[1]= -(self.setpoint[self.current_setpoint_index][1] - self.drone_position[1])
            self.cmd.rcPitch= int(1500 + self.error[1]*self.Kp[1]  +  (self.error[1]- self.prev_error[1])*self.Kd[1] +self.sum_error[1]*self.Ki[1])
            self.sum_error[1]= self.sum_error[1] + self.error[1]

            if self.cmd.rcPitch >self.max_values[1]:
                self.cmd.rcPitch = self.max_values[1]

            elif self.cmd.rcPitch < self.min_values[1]:
                self.cmd.rcPitch =  self.min_values[1]

            # Check if the drone is within the error range in all coordinate axes

            # if all(abs(self.error[1] <= 0.2)):
            # 	self.current_setpoint_index += 1     # Move to the next setpoint

        
        #------------------------------------------------------------------------------------------------------------

          #______along z axis    




            self.error[2]=  (self.drone_position[2] - self.setpoint[self.current_setpoint_index][2])  
            self.cmd.rcThrottle = int(1584.5 + self.error[2]*self.Kp[2] + (self.error[2] - self.prev_error[2])*self.Kd[2]+self.sum_error[2]*self.Ki[2])
            self.sum_error[2] = self.sum_error[2] + self.error[2] 
            if self.cmd.rcThrottle >self.max_values[2]:
                self.cmd.rcThrottle = self.max_values[2]
            elif self.cmd.rcThrottle <  self.min_values[2]:
                self.cmd.rcThrottle = self.min_values[2]

            # Check if the drone is within the error range in all coordinate axes

            # if all(abs(self.error[2] <= 0.2)):
            # 	self.current_setpoint_index += 1     # Move to the next setpoint  

        #-------------------------------------------------------------------------------------------------------------------------------------------   
        # updating the  previous error 

            self.prev_error[0] = self.error[0]       #along x-axix error 

            self.prev_error[1] = self.error[1]       #along y-axis errror

            self.prev_error[2] = self.error[2] 
            
            def disarm(self):
               self.cmd.rcAUX4 = 1100
               self.command_pub.publish(self.cmd)
               rospy.sleep(1)      #along z-axis error

        #------------------------------------------------------------------------------------------------------------       
            #    publishing the command

            self.command_pub.publish(self.cmd)
            self.alt_error_pub.publish(self.error[2])
            self.pitch_error_pub.publish(self.error[1])
            self.pitch_error_pub.publish(self.error[0])

            

            
            # Check if the drone is within the error range in all coordinate axes

            if abs(self.error[0]) <= 0.5 and abs(self.error[1]) <= 0.5 and abs(self.error[2]) <= 0.5: 
                self.current_setpoint_index += 1

            if self.drone_position == self.setpoint[len(self.setpoint)-1]:
                disarm()
     #------------------------------------------------------------------------------------------------------------      
    
    # Implmenting the image-processing to find the Nmber of led, area and Cordinate
    def detect_oraganism(self, image):
        # load the image, 
        # image = cv2.imread(image , 1)
        
        # convert it to grayscale, and blur it
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        
        # threshold the image to reveal light regions in the blurred image
        thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
        
        # perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=2)
        
        
        # perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
        labels = measure.label(thresh, connectivity=2, background=0)
        mask = np.zeros(thresh.shape, dtype="uint8")
        
        
        # loop over the unique components
        for label in np.unique(labels):
            # if this is the background label, ignore it
            if label == 0:
                continue
            
            # otherwise, construct the label mask and count the number of pixels 
            labelMask = np.zeros(thresh.shape, dtype="uint8")
            labelMask[labels == label] = 255
            numPixels = cv2.countNonZero(labelMask)
        
            # if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
            if numPixels > 100:
                mask = cv2.add(mask, labelMask)
        
        
            
        # find the contours in the mask, then sort them from left to right
        cnts  = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cnts  = imutils.grab_contours(cnts)
        # cnts, _  = contours.sort_contours(cnts)[0]


        # Chek the length of the tuple returned by grab_contours
        # if len(cnts) == 2:
        #     contours = cnts[0]
        # elif len(cnts) == 3:
        #     contours = cnts[1]
        # else:
        #     ValueError("Unexpected number of value returned by imputils")
        
        cnts = sorted(cnts,key=cv2.contourArea, reverse=True)[:2]
        # cnts, _  = contours.sort_contours(cnts)[0]
        
       
        
        
        # Initialize lists to store centroid coordinates and area
        centroid_coordinates = []
        areas = []
        num_leds = len(cnts)
        # print(num_leds)
        # rospy.loginfo(num_leds)
        # rospy.loginfo(centroid_coordinates)
        # print(centroid_coordinates)
        
        # Loop over the contours
        for i, c in enumerate(cnts):
        
        
        
            # Calculate the area of the contour
            area = cv2.contourArea(c)
        
            
        
            # Draw the bright spot on the image
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = float(M["m10"] / M["m00"])
                cY = float(M["m01"] / M["m00"])
                cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                cv2.circle(image, (int(cX), int(cY)), 7, (255, 0, 0), -1)
        
            # Append centroid coordinates and area to the respective lists
                centroid_coordinates.append((cX,cY))
                rospy.loginfo(centroid_coordinates)
                rospy.loginfo(num_leds)
                areas.append(area)
        return num_leds, centroid_coordinates

    # Cheling the type of oraganism
    def organism_type(self, num_leds):
        if num_leds==2:
            rospy.loginfo("alien_a")
            print("alien_a")
           
            return "alien_a"
        elif num_leds==3:
            rospy.loginfo("alien_b")
            return "alien_b"
        elif num_leds==4:
            rospy.loginfo("alien_b")
            print("alien_c")
            return "alien_c"
        else:
            return "unknown"
    
    def main(self):
        # Subscribing the image_callback
        rospy.Subscriber('/swift/camera_rgb/image_raw', Image, self.image_callback)

        while not rospy.is_shutdown():
            if self.image is not None:
                # Detect organism type based on the number of LEDs
                num_leds, centroid_coordinates = self.detect_oraganism(self.image)

                # identify organism type based on number of led
                organism= self.organism_type(num_leds)
        
                if organism!= "unknown" :#and centroid_coordinates:
                    # Calculate the avareage centroid Cordinates
                    avg_centroid = np.mean(np.array(centroid_coordinates), axis=0 )
        
                    # Ros Publisher
                    biolocation_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=10)
        
                    # Create Bioloction message
                    biolocation_msg = Biolocation()
                    biolocation_msg.organism_type= organism
                    biolocation_msg.whycon_x, biolocation_msg.whycon_y = avg_centroid
        
                    # publish the message
                    biolocation_pub.publish(biolocation_msg)
        
                    # Display information 


                    print(f"Organism Type: {organism}")
                    print(f"Centroid Cordinates: {avg_centroid}")
                    rospy.loginfo(organism)
                    rospy.loginfo(avg_centroid)
                

                else:
                    # print("NO LEDs detected.")
                    rospy.loginfo("NO LEDs detected.")
# Specify rate in Hz based upon your desired PID sampling time, i.e., if the desired sample time is 33ms, specify rate as 30Hz
            rospy.sleep(0.1) 
        
                
 #-----------------------------------------------------------------------------------------------------------       
                                   

if __name__ == '__main__':
    
    swift_drone = swift()
    r = rospy.Rate(40) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz

    while not rospy.is_shutdown():
        swift_drone.pid()
        r.sleep()
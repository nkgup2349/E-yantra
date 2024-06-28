
'''
# Team ID:          1168
# Theme:            Luminosity Drone
# Author List:      Mohammad Amanullah 
# Filename:         LD_1168_led_detection.py
# Functions:        None
# Global variables: None
'''



# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2

# load the image, 
image = cv2.imread('led.jpg', 1)

# convert it to grayscale, and blur it
gray = cv2.imread('led.jpg',0)
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
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)
cnts = contours.sort_contours(cnts)[0]

# loop over the contours


# Initialize lists to store centroid coordinates and area
centroid_coordinates = []
areas = []

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
        areas.append(area)
        

# Save the output image as a PNG file
cv2.imwrite("led_detection_results.png", image)

# Open a text file for writing
with open("led_detection_results.txt", "w") as file:
    # Write the number of LEDs detected to the file
    num_leds_detected = len(cnts)

    file.write(f"No. of LEDs detected: {num_leds_detected}\n")
    # Loop over the contours
    for i, (cX, cY) in enumerate(centroid_coordinates):
       
        # Write centroid coordinates and area for each LED to the file
        file.write(f"Centroid #{i + 1}: ({cX}, {cY})\nArea #{i + 1}: {area}\n")
        # file.write(f"Area #{i + 1}: {areas[i]}\n")
        # file.write(f"Centroid #{i + 1}: {cX}\nArea #{i + 1}: {cY}\n")

# Close the text file
file.close()

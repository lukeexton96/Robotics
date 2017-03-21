#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
 
wheel_radius = 0.05
robot_radius = 0.25

class Follower:
  def __init__(self):
    rospy.init_node('follower')
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    
    colours = {'Green': False, 'Blue': False, 'Yellow': False, 'Red': False}
    
    greenFound = False
    blueFound = False
    yellowFound = False
    redFound = False
    
## For SIMULATION 
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
    self.infrared_camera = rospy.Subscriber('/turtlebot/scan', LaserScan, self.laserRange)    
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=1)

    self.twist = Twist()
    self.laser.ranges = []
    self.laser = LaserScan()
    
    self.upperBoundColours = [[50,  100,  100], [120,  100,  100], [30,  100,  100], [0,  100,  100]]
    self.lowerBoundColours = [[180, 255, 255], [140, 255, 250], [255, 255, 255], [5, 255, 255]]
    self.masks = []
   
#   Upper and Lower bounds defined as
#   [0] = Green
#   [1] = Blue
#   [2] = Yellow
#   [3] = Red
    
  def laserRange(self, data):
      self.laser = data
    
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


## Check if colours have been found    
    ## If Green is false, append to mask for view
    if greenFound == False:
        lower_green = numpy.array(self.upperBoundColours[0])
        upper_green = numpy.array(self.lowerBoundColours[0])
        
        maskGreen = cv2.inRange(hsv, lower_green, upper_green)
    
    ## if Blue is false, append to mask for view
    if blueFound == False:
        
        lower_blue = numpy.array(self.upperBoundColours[1])
        upper_blue = numpy.array(self.lowerBoundColours[1])
        
        maskBlue = cv2.inRange(hsv, lower_blue, upper_blue)    
    
    ## if Yellow is false, append to mask for view
    if yellowFound == False:
        
        lower_yellow = numpy.array(self.upperBoundColours[2])
        upper_yellow = numpy.array(self.lowerBoundColours[2])
        
        maskYellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    ## if Red is false, append to mask for view
    if redFound == False:
            
        lower_red = numpy.array(self.upperBoundColours[3])
        upper_red = numpy.array(self.lowerBoundColours[3])
        
        maskRed = cv2.inRange(hsv, lower_red, upper_red)
        
    ## Print message if NO coloured monuments found
    if greenFound and blueFound and yellowFound and redFound == False:
        print "No monuments found."
        
    ## Print message if ALL coloured monuments found
    if greenFound and blueFound and yellowFound and redFound == True:
        print "All monuments found. End of session."  

    
## Define colour 'Blue' Identifiers
    lower_blue = numpy.array([ 120,  100,  100])
    upper_blue = numpy.array([140, 255, 250])
    maskBlue = cv2.inRange(hsv, lower_blue, upper_blue) 
    
## Define colour 'Yellow' Identifiers
    lower_yellow = numpy.array([ 30,  100,  100])
    upper_yellow = numpy.array([255, 255, 255])
    maskYellow = cv2.inRange(hsv, lower_yellow, upper_yellow) 

## Define colour 'red' Identifiers
    lower_red = numpy.array([ 0,  100,  100])
    upper_red = numpy.array([5, 255, 255])
    maskRed = cv2.inRange(hsv, lower_red, upper_red) 

## Set mask to contain only the colours defined above
    masks = maskGreen + maskBlue + maskYellow + maskRed
    
    h, w, d = image.shape
    
## Cut light
    search_top = 3*h/4 - 175
    search_bottom = 3*h/4 + 50
    masks[0:search_top, 0:w] = 0
    masks[search_bottom:h, 0:w] = 0
    
## Trying to see colours on screen 
    M = cv2.moments(masks)

    Distance = self.laser.ranges
    
## Check colour and check if false or true
    
    if M['m00'] > 0:
      if min(Distance) > 1 or math.isnan(min(Distance)):  
          
        ## Centre 'x' pixel 
          cx = int(M['m10']/M['m00'])
        ## Centre 'y' pixel
          cy = int(M['m01']/M['m00'])
        ## Draw cirlce on image
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          
        ## BEGIN CONTROL
          err = cx - w/2
          self.twist.linear.x = 0.6
          self.twist.angular.z = -float(err) / 100
          self.cmd_vel_pub.publish(self.twist)
      #    self.whatColour()
          
# END CONTROL
    cv2.imshow("window", image)
    cv2.imshow("window", masks)
    cv2.waitKey(1)   
    
    # Class selector used to run class
if __name__ == "__main__":
    W=Follower()
    rospy.spin()    
    
    
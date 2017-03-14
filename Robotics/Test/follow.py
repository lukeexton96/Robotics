#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
 
wheel_radius = 0.05
robot_radius = 0.25
     
class Follower:
    def __init__(self):
        rospy.init_node('follower')
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('/turtlebot_1/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/turtlebot_1/cmd_vel', Twist, queue_size=1)
        self.infrared_camera = rospy.Subscriber('/turtlebot_1/scan', LaserScan, self.laserRange)
#    self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
#    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#    self.infrared_camera = rospy.Subscriber('/scan', LaserScan, self.laserRange)
        self.twist = Twist()

    def laserRange(self, data):
        self.laser = data

    def spin(self):          
        self.twist.linear.x = 0.0
        self.twist.angular.z = math.pi/2
        self.cmd_vel_pub.publish(self.twist)
    
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = numpy.array([ 50,  100,  0])
        upper_green = numpy.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        Distance = self.laser.ranges
        print min(Distance)
        if M['m00'] > 0:
            if Distance[(len(Distance)/2)] > 1 or Distance.isnan(Distance[(len(Distance)/2)]): 
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                # BEGIN CONTROL
                err = cx - w/2
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err) / 100
                self.cmd_vel_pub.publish(self.twist)
                # END CONTROL
                cv2.imshow("window", image)
                cv2.imshow("window", mask)
                cv2.waitKey(1)
            #else:
           

if __name__ == "__main__":
    W=Follower()
    W.spin()
    rospy.spin()    
    
    

#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# red (s and v in percentage)
# 0, 50, 50: 0, 190, 100
# 16, 100, 100: 4, 255, 255

# green
# 90, 50, 50
# 137, 100, 100

# blue
# 190, 50, 50
# 252, 100, 100

# yellow
# 40
# 67

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    # cv2.namedWindow("yellow_mask", 1)
    cv2.namedWindow("green_mask", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.orientation = -1
    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  
    # yellow
    lower_yellow = numpy.array([ 20,  40, 40])
    upper_yellow = numpy.array([40, 255, 250])

    # green
    lower_green = numpy.array([40, 40, 40])
    upper_green = numpy.array([86, 255, 250])

    # blue
    lower_blue = numpy.array([190, 40, 40])
    upper_blue = numpy.array([252, 255, 250])

    # red
    lower_red = numpy.array([0, 40, 40])
    upper_red = numpy.array([4, 255, 250])

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    
    # erase pixels outside region of interest
    mask_yellow[0:search_top, 0:w] = 0
    mask_yellow[search_bot:h, 0:w] = 0

    mask_green[0:search_top, 0:w] = 0
    mask_green[search_bot:h, 0:w] = 0

    M_yellow = cv2.moments(mask_yellow)
    M_green = cv2.moments(mask_green)
  
    # if M_green['m00'] > 0:
    #   print("change oreintation")
    #   self.orientation = 1.0
    #   cx = int(M_green['m10']/M_green['m00'])
    #   cy = int(M_green['m01']/M_green['m00'])
    #   cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

    #   if mask_green[cy][cx] > 0:
    #     # BEGIN CONTROL
    #     err = cx - w/2
    #     self.twist.linear.x = 0.4
        
    #     self.twist.angular.z = 1.0 * float(err) / 100
    #     self.cmd_vel_pub.publish(self.twist)
    #     # END CONTROL

    if M_yellow['m00'] > 0:
      cx = int(M_yellow['m10']/M_yellow['m00'])
      cy = int(M_yellow['m01']/M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.8
      
      self.twist.angular.z = self.orientation * float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
      
    # cv2.imshow("yellow_mask", mask_yellow)
    cv2.imshow("green_mask", mask_green)
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
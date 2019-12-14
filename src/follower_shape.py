#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import argparse

class Color:
  RED = 1
  GREEN = 2
  BLUE = 3
  YELLOW = 4

class Follower:
  def __init__(self, color=None):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    cv2.namedWindow("yellow_mask", 1)
    cv2.namedWindow("red_mask", 1)

    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback_shape)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.orientation = -1
    self.twist = Twist()
    self.stop = False

  def image_callback_shape(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  
    # yellow
    lower_yellow = numpy.array([ 25,  40, 40])
    upper_yellow = numpy.array([35, 255, 250])

    # red
    lower_red = numpy.array([0, 40, 40])
    upper_red = numpy.array([15, 255, 250])

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 40
    
    # erase pixels outside region of interest
    mask_yellow[0:search_top, 0:w] = 0
    mask_yellow[search_bot:h, 0:w] = 0

    mask_red[0:search_top, 0:w] = 0
    mask_red[search_bot:h, 0:w] = 0

    M_yellow = cv2.moments(mask_yellow)
    M_red = cv2.moments(mask_red)

    if M_red['m00'] > 0:
      cnts_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, 
        cv2.CHAIN_APPROX_SIMPLE)[-2]
      cx = int(M_red['m10']/M_red['m00'])
      cy = int(M_red['m01']/M_red['m00'])
      cv2.circle(image, (cx, cy), 20, (255,0,0), -1)
      
      for idx, cnt in enumerate(cnts_red):
        # calculate number of sides
        peri = cv2.arcLength(cnt, True) # finds the Contour Perimeter
        approx = cv2.approxPolyDP(cnt, 0.05 * peri, True)

        if len(approx) > 3:
          print("it might be a star...")
          self.stop = True
      
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.8
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    
    elif M_yellow['m00'] > 0:
      cx = int(M_yellow['m10']/M_yellow['m00'])
      cy = int(M_yellow['m01']/M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

      if not self.stop:
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.8
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
      else:
        self.cmd_vel_pub.publish(Twist())
            
    cv2.imshow("window", image)
    cv2.imshow("yellow_mask", mask_yellow)
    cv2.imshow("red_mask", mask_red)
    cv2.moveWindow("window", 1200, 50)
    cv2.moveWindow("yellow_mask", 500, 600)
    cv2.moveWindow("red_mask", 1200, 600)
    cv2.waitKey(3)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--color', help="Inter color, RED=1, GREEN=2, BLUE=3")

  rospy.init_node('follower')
  follower = Follower()
  rospy.spin()
# END ALL

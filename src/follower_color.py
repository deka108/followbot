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

    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback_color)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.orientation = -1
    self.twist = Twist()
    self.cur_color = color
    self.red_regions = 0
    self.move_before_stopping = 0
    self.stop = False

  def image_callback_color(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  
    # yellow
    lower_yellow = numpy.array([ 25,  40, 40])
    upper_yellow = numpy.array([35, 255, 250])

    # green
    lower_green = numpy.array([40, 40, 40])
    upper_green = numpy.array([75, 255, 250])

    # blue
    lower_blue = numpy.array([110, 40, 40])
    upper_blue = numpy.array([130, 255, 250])

    # red
    lower_red = numpy.array([0, 40, 40])
    upper_red = numpy.array([15, 255, 250])

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

    mask_blue[0:search_top, 0:w] = 0
    mask_blue[search_bot:h, 0:w] = 0

    mask_red[0:search_top, 0:w] = 0
    mask_red[search_bot:h, 0:w] = 0

    M_yellow = cv2.moments(mask_yellow)
    M_green = cv2.moments(mask_green)
    M_blue = cv2.moments(mask_blue)
    M_red = cv2.moments(mask_red)

    if M_green['m00'] > 0:
      print("detect green")
      cx = int(M_green['m10']/M_green['m00'])
      cy = int(M_green['m01']/M_green['m00'])
      cv2.circle(image, (cx, cy), 20, (255,0,0), -1)

      err = cx - w/2
      self.twist.linear.x = 0.4
      self.twist.angular.z = 0
      self.cmd_vel_pub.publish(self.twist)
      self.cur_color = Color.GREEN

    elif M_blue['m00'] > 0:
      print("detect blue")
      cx = int(M_blue['m10']/M_blue['m00'])
      cy = int(M_blue['m01']/M_blue['m00'])
      cv2.circle(image, (cx, cy), 20, (0,255,0), -1)

      err = cx - w/2
      self.twist.linear.x = 0.4
      self.twist.angular.z = 0
      self.cmd_vel_pub.publish(self.twist)
      self.cur_color = Color.BLUE
      
    elif M_yellow['m00'] > 0:
      cx = int(M_yellow['m10']/M_yellow['m00'])
      cy = int(M_yellow['m01']/M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      
      # count how many red regions are seen
      if M_red['m00'] > 0:
        self.red_regions += 1
      
      # completely seen all the red regions
      if M_red['m00'] == 0 and self.red_regions > 15:
        self.move_before_stopping += 1

        # move past the red region
        if self.move_before_stopping > 10:
          self.stop = True

      if self.stop:
        print("STOP")
        self.cmd_vel_pub.publish(Twist())
      else:
        if mask_yellow[cy][cx] > 0:
          # BEGIN CONTROL
          err = cx - w/2
          self.twist.linear.x = 0.8
          self.twist.angular.z = -float(err) / 100
          self.cmd_vel_pub.publish(self.twist)
          # END CONTROL
        else:
          cnts = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE)[-2]
      
          if len(cnts) > 0:
            left = None
            right = None

            # loop over the contours
            for c in cnts:
              # compute the center of the contour
              M = cv2.moments(c)
              if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])

                if left is None:
                  left = cX
                if right is None:
                  right = cX

                if cX < left:
                  left = cX

                if cX > right:
                  right = cX
          
          # choose left center
          if self.cur_color == Color.GREEN:
            print("CHOOSE LEFT BLOB")
            # BEGIN CONTROL
            err = left - w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            # END CONTROL
          elif self.cur_color == Color.BLUE:
            print("CHOOSE RIGHT BLOB")
            # BEGIN CONTROL
            err = right - w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            # END CONTROL
          else:
            # BEGIN CONTROL
            err = cx - w/2
            self.twist.linear.x = 0.8
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            # END CONTROL
            
    cv2.imshow("window", image)
    # cv2.moveWindow("window", 1000, 50)
    cv2.waitKey(3)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--color', help="Inter color, RED=1, GREEN=2, BLUE=3")

  rospy.init_node('follower')
  follower = Follower()
  rospy.spin()
# END ALL

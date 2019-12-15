#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import argparse
import numpuy as np

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

    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.orientation = -1
    self.twist = Twist()
    self.stop = False
  def predict_dir(self, mask,start,end):
    wd = mask.shape[1]
    wdstart = int(wd/2) - 30
    wdend = int(wd/2) + 30
    center = mask[start:end,wdstart:wdend]
    wdstart = 0
    wdend = 60
    left = mask[start:end, wdstart:wdend]
    wdstart = wd-60
    wdend = wd

    right = mask[start:end, wdstart:wdend]
    lc,rc = np.where(center == 0) 
    ll,rl = np.where(left == 0)
    lr,rr = np.where(right == 0)

    if len(lc) == 0:
      print("Not diverging")
      if len(ll) > 0:
        print("Turn left")
      if len(lr) > 0:
        print("Turn right")
    
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  
    # yellow
    lower_yellow = numpy.array([ 25,  40, 40])
    upper_yellow = numpy.array([35, 255, 250])

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 75
    
    # erase pixels outside region of interest
    mask_yellow[0:search_top, 0:w] = 0
    mask_yellow[search_bot:h, 0:w] = 0

    M_yellow = cv2.moments(mask_yellow)

    if M_yellow['m00'] > 0:
      cx = int(M_yellow['m10']/M_yellow['m00'])
      cy = int(M_yellow['m01']/M_yellow['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      cv2.circle(image, (20, cy), 20, (0,255,0), -1)
      cv2.circle(image, (w-20, cy), 20, (0,255,0), -1)
      
      self.predict_dir(mask_yellow, search_top, search_bot)
      cnts = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, 
        cv2.CHAIN_APPROX_SIMPLE)[-2]

      for idx, cnt in enumerate(cnts):
        # calculate number of sides
        peri = cv2.arcLength(cnt, True) # finds the Contour Perimeter
        approx = cv2.approxPolyDP(cnt, 0.05 * peri, True)
        
        if M_yellow['m00'] > 2500000:
            print("yellow blob area: {}".format(M_yellow['m00']))
            print("#cnts: {}, cnt: {}, #sides: {}, area: {}, center is yellow: {}".format(
                len(cnts), 
                idx, 
                len(approx), 
                cv2.contourArea(cnt),
                mask_yellow[cy][cx]))

      if not self.stop:
        # slow down
        if M_yellow['m00'] > 2500000:
            print("slow down")
            # BEGIN CONTROL
            err = cx - w/2
            self.twist.linear.x = 0.3
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
      else:
        self.cmd_vel_pub.publish(Twist())
            
    cv2.imshow("window", image)
    cv2.imshow("yellow_mask", mask_yellow)
    cv2.moveWindow("window", 1200, 50)
    cv2.moveWindow("yellow_mask", 1200, 600)
    
    cv2.waitKey(3)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--color', help="Inter color, RED=1, GREEN=2, BLUE=3")

  rospy.init_node('follower')
  follower = Follower()
  rospy.spin()
# END ALL

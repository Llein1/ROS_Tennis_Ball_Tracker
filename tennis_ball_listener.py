#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class TennisBallListener:
    def __init__(self):
    
        self.bridge = CvBridge()
                
        self.image_sub = rospy.Subscriber('tennis_ball_image', Image, self.image_callback) # 'tennis_ball_image' adlı görüntü konusunu dinleyen bir ROS abonesi oluşturulur.

    def image_callback(self, msg):
        try:          
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8") # ROS görüntü verisini OpenCV formatına dönüştürme işlemi yapılır.
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Tenis topu rengi için alt ve üst sınırlar tanımlanır (HSV renk uzayında).
        yellowLower = (30, 150, 100)
        yellowUpper = (50, 255, 255)

        
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Görüntü HSV renk uzayına dönüştürülür.
        
        
        mask = cv2.inRange(hsv_image, yellowLower, yellowUpper) # Renk sınırları arasındaki pikselleri beyaz, diğerlerini siyah yapan bir maske oluşturulur.
        
        
        contours, hierarchy = cv2.findContours(mask.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)# Contour'ları bulma işlemi yapılır.
        
        
        black_image = np.zeros([mask.shape[0], mask.shape[1],3],'uint8') # Siyah bir arkaplan üzerinde Contour'ları çizmek için boş bir görüntü oluşturulur.

        for c in contours:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c, True)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            
            
            if (area > 100): # Contour'ları alanı belirli bir eşik değerinden büyükse, çizim işlemleri yapılır.
                cv2.drawContours(frame, [c], -1, (150,250,150), 1)
                cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
                M = cv2.moments(c)
                cx = -1
                cy = -1
                if (M['m00'] != 0):
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), int(radius), (0,0,255), 1)
                cv2.circle(black_image, (cx, cy), int(radius), (0,0,255), 1)
                cv2.circle(black_image, (cx, cy), 5, (150,150,255), -1)
                print("Area: {}, Perimeter: {}".format(area, perimeter))
        
        print("Number of contours: {}".format(len(contours)))
        
        cv2.imshow("RGB Image Contours", frame)
        cv2.imshow("Black Image Contours", black_image)
        cv2.imshow("Tennis Ball Detection", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    
    rospy.init_node('tennis_ball_listener', anonymous=True) # ROS düğümünü başlatır.
    
    listener = TennisBallListener()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


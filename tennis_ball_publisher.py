#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class TennisBallPublisher:
    
    def __init__(self):
        
        rospy.init_node('tennis_ball_publisher', anonymous=True) # ROS düğümünü başlatır
        
        self.image_pub = rospy.Publisher('tennis_ball_image', Image, queue_size=10)
        
        self.bridge = CvBridge() # OpenCV görüntülerini ROS Image mesajlarına dönüştürmek için köprü oluştur

    def publish_frames(self, video_path):
        
        cap = cv2.VideoCapture(video_path) # Video dosyasını açar

        if not cap.isOpened():
            rospy.logerr("Video akışı açılırken hata oluştu")
            return

        rate = rospy.Rate(30)  
        
        while not rospy.is_shutdown():           
            ret, frame = cap.read() # Video dosyasından kare okur
            
            if not ret:
                rospy.loginfo("Video akışı sona erdi")
                break

            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8") # OpenCV verisini CvBridge kullanarak ROS Image mesajına dönüştürür
                
                self.image_pub.publish(image_msg) # Oluşturulan Image mesajını 'tennis_ball_image' adındaki yayına gönderir
                rate.sleep()
            except CvBridgeError as e:
                rospy.logerr(e)
        
        cap.release()

if __name__ == '__main__':
    try:
        video_path = "/home/llein/catkin_ws/src/ROS_Tennis_Ball_Tracker.mp4"
        
        publisher = TennisBallPublisher()
        
        publisher.publish_frames(video_path) # Video karelerini yayınlar
    except rospy.ROSInterruptException:       
        pass


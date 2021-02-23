#!/usr/bin/python

import cv2,rospy,time 
import math
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

x,y,theta=0,0,0
bridge=CvBridge()
cv_image=np.empty(shape=[0])

black_color=(0,0,0)

lower_green=(35,100,100)
upper_green=(70,255,255)



def callback(img_data):
	global bridge
	global cv_image
	cv_image=bridge.imgmsg_to_cv2(img_data,"bgr8")
def callback_pose(data):
	global x
	global y
	global theta
	
	x=data.x
	y=data.y
	theta=data.theta

def go_to_goal(x_goal,y_goal):
	global x
	global y
	global theta

	velmsg=Twist()

	time.sleep(0.01)

	while True:
		
		distance=0.5*abs(math.sqrt(((x_goal-x)**2)+((y_goal-y)**2)))
		
		desired_angle_goal=math.atan2(y_goal-y,x_goal-x)
		angular_speed=(desired_angle_goal-theta)*4.0

		velmsg.linear.x=distance
		
		velmsg.angular.z=angular_speed
		

		pub.publish(velmsg)
		
		if distance<1.0:
			break
	

if __name__=="__main__":
	rospy.init_node("dfdf")
	rospy.Subscriber("/usb_cam/image_raw",Image,callback)
	rospy.Subscriber('/turtle1/pose',Pose,callback_pose)
	pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=1)
	
	time.sleep(1)
	
	while not rospy.is_shutdown():
		img=cv2.line(cv_image,(0,160),(640,160),black_color,3)
		img=cv2.line(cv_image,(0,320),(640,320),black_color,3)
		img=cv2.line(cv_image,(213,0),(213,480),black_color,3)
		img=cv2.line(cv_image,(427,0),(427,480),black_color,3)
	
		img_hsv=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
		img_mask=cv2.inRange(img_hsv,lower_green,upper_green)

		kernel=np.ones((11,11),np.uint8)
		img_mask=cv2.morphologyEx(img_mask,cv2.MORPH_OPEN,kernel)
		img_mask=cv2.morphologyEx(img_mask,cv2.MORPH_CLOSE,kernel)		

		img_result=cv2.bitwise_and(cv_image,cv_image,mask=img_mask)
		
		
		numOfLabels,img_label,stats, centroids=cv2.connectedComponentsWithStats(img_mask)
		
		for idx, centroid in enumerate(centroids):
			if stats[idx][0]==0 and stats[idx][1]==0:
				continue
			

			x,y,width,height,area=stats[idx]
			centerX,centerY=int(centroid[0]),int(centroid[1])
		
			
				
			if centerX>70 and centerX<140 and centerY>60 and centerY<100:
				go_to_goal(1,10)
			elif centerX>280 and centerX<350 and centerY>60 and centerY<100:
				go_to_goal(5,10)
			elif centerX>490 and centerX<570 and centerY>60 and centerY<100:
				go_to_goal(10,10)
			elif centerX>70 and centerX<140 and centerY>220 and centerY<260:
				go_to_goal(1,5)
			elif centerX>280 and centerX<350 and centerY>220 and centerY<260:
				go_to_goal(5,5)
			elif centerX>490 and centerX<570 and centerY>220 and centerY<260:
				go_to_goal(10,5)
			elif centerX>70 and centerX<140 and centerY>380 and centerY<420:
				go_to_goal(1,1)
			elif centerX>280 and centerX<350 and centerY>380 and centerY<420:
				go_to_goal(5,1)
			elif centerX>490 and centerX<570 and centerY>380 and centerY<420:
				go_to_goal(10,1)

		
		
		cv2.imshow("camera",img_result)
		cv2.imshow("camera1",cv_image)
		


		if cv2.waitKey(1) & 0xff == ord("q"):
			break
	cv2.destroyALLWindows()


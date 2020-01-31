
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
from std_msgs.msg import Int8
import std_msgs
import pdb

global status
status = 1


data_status = Int8()
data_status.data = 1
#model = keras_segmentation.pretrained.pspnet_101_voc12()
# Instantiate CvBridge
bridge = CvBridge()
i = 0

def callback(data):
	global status
	status = data.data

if __name__ == '__main__':

	rospy.init_node('writerNode', anonymous=True)

	rospy.Subscriber("state",Int8,callback)
	pub = rospy.Publisher('image_classified',Image,queue_size=10)
	pub2 = rospy.Publisher('state',Int8,queue_size=20)

	rate = rospy.Rate(0.5)

	h = std_msgs.msg.Header()
	while(not rospy.is_shutdown()):
		global status
		if(status == 3):

			image_cv2 = im = cv2.imread("/home/jsm/.virtualenvs/cv/lib/python3.5/site-packages/im.jpg")
			try:
#         Convert your ROS Image message to OpenCV2
				cv2_img = bridge.cv2_to_imgmsg(image_cv2, encoding="bgr8")
			except CvBridgeError, e:
				print(e + "yes")
#         Save your OpenCV2 image as a jpeg 
			if(cv2_img is not None):
				h.stamp = rospy.Time.now()
				h.frame_id = 'kinect2_rgb_optical_frame'

				cv2_img.header = h
				pub.publish(cv2_img)
				pub2.publish(data_status)
				rate.sleep()



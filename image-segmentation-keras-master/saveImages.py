import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pdb

global status
status = 1
# Instantiate CvBridge
bridge = CvBridge()
i = 0
data_status = Int8()
data_status.data = 2


def callback(data):
	global status
	status = data.data

if __name__ == '__main__':

	rospy.init_node('savingImagesNode', anonymous=True)

	rospy.Subscriber("state",Int8,callback)

	pub = rospy.Publisher('state',Int8,queue_size=20)
	rate = rospy.Rate(0.5)

	while(not rospy.is_shutdown()):
		i = i + 1
		global status
		if(status == 1):

			image_msg = rospy.wait_for_message("/kinect2/hd/image_color", Image)
	#         Save your OpenCV2 image as a jpeg 

			try:
	#         Convert your ROS Image message to OpenCV2
				cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
				cv2.imwrite('/home/jsm/.virtualenvs/cv/lib/python3.5/site-packages/camera_imag.jpg', cv2_img)
	#        cv2.waitKey(1000)
				pub.publish(data_status)
				status = 2
				rate.sleep()
			except CvBridgeError, e:
				print(e)

			



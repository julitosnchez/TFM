# USAGE
# python yolo.py --image images/baggage_claim.jpg --yolo yolo-coco

# import the necessary packages
import numpy as np
import argparse
import time
import rospy
from sensor_msgs.msg import Image as image_ros
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8
import cv2
import os
from io import StringIO,BytesIO
import PIL
from PIL import Image
from skimage import io

global status
status = 1

def callback(data):
	global status
	status = data.data

data_status = Int8()
data_status.data = 3

bridge = CvBridge()

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-y", "--yolo", required=True,
	help="base path to YOLO directory")
ap.add_argument("-c", "--confidence", type=float, default=0.7,
	help="minimum probability to filter weak detections")
ap.add_argument("-t", "--threshold", type=float, default=0.3,
	help="threshold when applyong non-maxima suppression")
args = vars(ap.parse_args())

# load the COCO class labels our YOLO model was trained on
labelsPath = os.path.sep.join([args["yolo"], "coco.names"])
LABELS = open(labelsPath).read().strip().split("\n")

# initialize a list of colors to represent each possible class label
np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
	dtype="uint8")



# derive the paths to the YOLO weights and model configuration
weightsPath = os.path.sep.join([args["yolo"], "yolov3.weights"])
configPath = os.path.sep.join([args["yolo"], "yolov3.cfg"])

# load our YOLO object detector trained on COCO dataset (80 classes)
print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
#net = cv2.dnn.readNet("yolov3.weights","yolov3.config")

#def callback(msg):
#	try:
#		#         Convert your ROS Image message to OpenCV2
#		cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
#	except CvBridgeError,e:
#		print(e)
#	else:
#		#         Save your OpenCV2 image as a jpeg 
#		detectObject(cv2_img)

def detectObject(image):
	(H, W) = image.shape[:2]
	# determine only the *output* layer names that we need from YOLO
	ln = net.getLayerNames()
	ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

	# construct a blob from the input image and then perform a forward
	# pass of the YOLO object detector, giving us our bounding boxes and
	# associated probabilities
	blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
		swapRB=True, crop=False)
	net.setInput(blob)
	start = time.time()
	layerOutputs = net.forward(ln)
	end = time.time()

	# show timing information on YOLO
	print("[INFO] YOLO took {:.6f} seconds".format(end - start))

	# initialize our lists of detected bounding boxes, confidences, and
	# class IDs, respectively
	boxes = []
	confidences = []
	classIDs = []

	# loop over each of the layer outputs
	for output in layerOutputs:
		# loop over each of the detections
		for detection in output:
			# extract the class ID and confidence (i.e., probability) of
			# the current object detection
			scores = detection[5:]
			classID = np.argmax(scores)
			confidence = scores[classID]

			# filter out weak predictions by ensuring the detected
			# probability is greater than the minimum probability
			if confidence > args["confidence"]:
				# scale the bounding box coordinates back relative to the
				# size of the image, keeping in mind that YOLO actually
				# returns the center (x, y)-coordinates of the bounding
				# box followed by the boxes' width and height
				box = detection[0:4] * np.array([W, H, W, H])
				(centerX, centerY, width, height) = box.astype("int")

				# use the center (x, y)-coordinates to derive the top and
				# and left corner of the bounding box
				x = int(centerX - (width / 2))
				y = int(centerY - (height / 2))

				# update our list of bounding box coordinates, confidences,
				# and class IDs
				boxes.append([x, y, int(width), int(height)])
				confidences.append(float(confidence))
				classIDs.append(classID)

	# apply non-maxima suppression to suppress weak, overlapping bounding
	# boxes
	idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"],
		args["threshold"])

	# ensure at least one detection exists
	if len(idxs) > 0:
		overlay = np.zeros(shape=[1080, 1920, 3], dtype=np.uint8) 
		# loop over the indexes we are keeping
		for i in idxs.flatten():
			# extract the bounding box coordinates
			(x, y) = (boxes[i][0], boxes[i][1])
			(w, h) = (boxes[i][2], boxes[i][3])

#			overlay = image.copy()

			# draw a bounding box rectangle and label on the image
			color = [int(c) for c in COLORS[classIDs[i]]]

#			print("COLOR: " + str(color))
#			print("X: " + str(x) + " Y: " + str(y) + " W: " + str(w) + " H: " + str(h)) 
			cv2.rectangle(overlay, (x, y), (x + w, y + h), color, -1)

			
			alpha = 1

#			print(image.shape)
			image = cv2.addWeighted(overlay,alpha,image,0,0)

			text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
			cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, color, 2)
	else:
		image = np.zeros(shape=[1080, 1920, 3], dtype=np.uint8)
#			print("OJO: " + str(image[y+3,x+4]))

	# show the output image
#	num_point = 0
#	for i in range(image.shape[0]):
#		for j in range(image.shape[1]):
#			if(image[i,j][0] == 151 and image[i,j][1] == 62):
#				num_point = num_point + 1
#	print(num_point)
	cv2.imwrite("./im.jpg",image)
#	cv2.waitKey(3000)

if __name__=="__main__":
	rospy.init_node('classifyingNode', anonymous=True)

	rospy.Subscriber("state",Int8,callback)

	pub = rospy.Publisher('state',Int8,queue_size=20)
	rate = rospy.Rate(0.5)

	while(not rospy.is_shutdown()):
		global status

		if(status == 2):
#			try:
				im = cv2.imread("./camera_imag.jpg")
				detectObject(im)
				pub.publish(data_status)
				status = 3
				rate.sleep()

#			except:
#				print("Error leyendo imagen de la Kinect")

		






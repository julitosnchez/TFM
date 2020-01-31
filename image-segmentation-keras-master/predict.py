import keras_segmentation
import rospy
import keras
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import pdb

keras.backend.clear_session()
model = keras_segmentation.pretrained.pspnet_50_ADE_20K() # load the pretrained model trained on ADE20k dataset
#model = keras_segmentation.pretrained.pspnet_101_voc12()
# Instantiate CvBridge
bridge = CvBridge()
i = 0

def predict(img):

    out = model.predict_segmentation(
        inp=img,
        out_fname="outJulio1.png")

def callback(msg):
    
    global i
    global model

    print(i)
     
    print("Received an image!")
    try:
#         Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
#         Save your OpenCV2 image as a jpeg 
        cv2.imwrite('./camera_image' + str(i) +'.jpg', cv2_img)


#    pdb.set_trace()
#    model.predict_segmentation(
#        inp='./camera_image' + str(i) +'.jpg',
#        out_fname='./out' + str(i) +'.jpg'
#    )


#    i = i + 1
    



if __name__ == '__main__':




   rospy.init_node('listener', anonymous=True)

#   rospy.Subscriber("/kinect2/hd/image_color", Image, callback)

##     spin() simply keeps python from exiting until this node is stopped
#   rospy.spin()

   while(True):

      image_msg = rospy.wait_for_message("/kinect2/hd/image_color", Image)

      try:
#         Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
      except CvBridgeError, e:
         print(e)
      else:
#         Save your OpenCV2 image as a jpeg 
        cv2.imwrite('./camera_imag.jpg', cv2_img)


      out = model.predict_segmentation(
        inp="camera_imag.jpg",
        out_fname="outJulio1.jpg")



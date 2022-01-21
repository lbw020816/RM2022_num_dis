#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from os.path import realpath, dirname, join
#from numPred.msg import out_msg
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers, Sequential
from tensorflow.keras import optimizers, losses, datasets
#import keras as K

class numPredNode(object):
    def __init__(self):
        super(numPredNode,self).__init__()
        weight_path="/home/catmult7/numTest/src/numPred/weights.ckpt.index"
        self.model=self.bulidModel(weight_path)
        #model.summary()
        rospy.init_node('num_predictor',anonymous=True)
        self._bridge = CvBridge()
        self.pub_num=rospy.Publisher('/numPred/num',Int16,queue_size=1)
        self.sub_image=rospy.Subscriber("/armor_detector/armor_roi",Image,self.img_callback)
        # self.sub_whole_image=rospy.Subscriber("/armor_detector/binary_img",Image,self.img_whole_callback)
        # self.sub_binary_image=rospy.Subscriber("/armor_detector/output_img",Image,self.img_binary_callback)
        print("initialize done")
        rospy.spin()


    # def img_whole_callback(self,data):
    #     try:
    #         # Convert image to numpy array
             #show_image = self._bridge.imgmsg_to_cv2(data, "bgr8")


    #     except CvBridgeError as e:
    #         rospy.logerr(e)

    #     cv2.imshow("output img",show_inetwork.s

    # def img_binary_callback(self,data):
    #     try:
    #         # Convert image to numpy array
    #         binary_image = self._bridge.imgmsg_to_cv2(data, "bgr8")


    #     except CvBridgeError as e:
    #         rospy.logerr(e)

    #     cv2.imshow("binary img",binary_image)
    #     key=cv2.waitKey(1)

    def img_callback(self, data):
        try:
            # Convert image to numpy array
            self.current_image = self._bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerr(e)

        num=self.pred_number(self.current_image)
        num=int(num)
        #print(num)
        pred_out=Int16()
        pred_out=num
        self.pub_num.publish(pred_out)

    def pred_number(self,img):
        img=cv2.resize(img,(28,28))
        img=tf.convert_to_tensor(img, tf.float16)
        testImg=tf.reshape(img,[1,28,28,1])
        
        #testImg_a=np.array(img)
        num=1000

        pred=self.model.predict(testImg)
        num = np.argmax(pred)
        return num

    def bulidModel(self, weight_path):
        network=Sequential([
        layers.Conv2D(32,kernel_size=3,strides=1,activation='relu'),
        layers.Conv2D(64,kernel_size=3,strides=1,activation='relu'),
        layers.MaxPooling2D(pool_size=2,strides=2),
        layers.Dropout(0.25),
        layers.Flatten(),
        layers.Dense(128, activation="relu"),
        layers.Dropout(0.5),
        layers.Dense(10,activation='softmax')
        ])
        network.compile(optimizers=optimizers.Adam(lr=0.01),loss=losses.CategoricalCrossentropy(from_logits=True), metrics=['accuracy'])
        network.load_weights(weight_path)
        network.build(input_shape=(1,28,28,1))
        print("loading model from"+weight_path)
        return network

def main():
    """ main function
    """
    node = numPredNode()

if __name__ == '__main__':
    main()

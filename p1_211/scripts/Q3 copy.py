#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division


# Para rodar, recomendamos que faça:
# 
#    roslaunch my_simulation circuito
#
# Depois para rodar
#
#    rosrun p1_211 Q3.py


import rospy 

import numpy as np

import cv2

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import math


ranges = None
minv = 0
maxv = 10

bridge = CvBridge()


## ROS
def scaneou(dado):

	global distancia

	ranges = np.array(dado.ranges).round(decimals=2)
	distancia = ranges[0]

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv2.imshow("Camera", cv_image)
        ##
        copia = cv_image.copy() # se precisar usar no while
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

if __name__=="__main__":

    rospy.init_node("q3")

    topico_imagem = "/camera/image/compressed"
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    pose_sub = rospy.Subscriber('/odom', Odometry , self.sub_callback)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)

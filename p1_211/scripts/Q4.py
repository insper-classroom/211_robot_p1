#! /usr/bin/env python3
# -*- coding:utf-8 -*-
from __future__ import print_function, division


# Sugerimos rodar com:
#         roslaunch my_simulation caixas.launch
# Depois para rodar este script
# rosrun p1_211 Q4.py

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from tf import transformations
import math


ranges = None
minv = 0
maxv = 10

bridge = CvBridge()

x = -1
y = -1
contador = 0
radianos = None
pula = 10

def recebe_odometria(data):
    global x
    global y
    global contador
    global radianos

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    # Angulos tem os angulos absolutos em graus
    angulos = np.degrees(transformations.euler_from_quaternion(lista))  
    # Angulos tem os angulos absolutos em radianos
    radianos =   transformations.euler_from_quaternion(lista)  

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1

def scaneou(dado):
    global ranges
    global minv
    global maxv
    # A variável ranges vai conter as leituras do laser
    ranges = np.array(dado.ranges).round(decimals=2)
    minv = dado.range_min 
    maxv = dado.range_max
 
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

if __name__=="__main__":

    rospy.init_node("q4")

    topico_imagem = "/camera/image/compressed"

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)




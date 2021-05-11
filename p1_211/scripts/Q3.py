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

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from scipy.spatial.transform import Rotation as R

import math
import Q3_utils as q3utils



ranges = None
minv = 0
maxv = 10

bridge = CvBridge()

def quart_to_euler(orientacao):
    """
    Converter quart. para euler (XYZ)
    Retorna apenas o Yaw (wz)
    """
    r = R.from_quat(orientacao)
    wx, wy, wz = (r.as_euler('xyz', degrees=True))

    return wz

## ROS
def mypose(msg):
    """
    Recebe a Leitura da Odometria.
    Para esta aplicacao, apenas a orientacao esta sendo usada
    """
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w

    orientacao_robo = [[x,y,z,w]]

def scaneou(dado):
    """
    Rebe a Leitura do Lidar
    Para esta aplicacao, apenas a menor distancia esta sendo usada
    """
    global distancia
    
    ranges = np.array(dado.ranges).round(decimals=2)
    distancia = ranges[0]



## Variáveis novas criadas pelo gabarito

centro_yellow = (320,240)
frame = 0
skip = 3
m = 0
angle_yellow = 0 # angulo com a vertical

low = q3utils.low
high = q3utils.high

## 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global centro_yellow
    global m
    global angle_yellow

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv2.imshow("Camera", cv_image)
        ##
        copia = cv_image.copy() # se precisar usar no while

        if frame%skip==0: # contamos a cada skip frames
            mask = q3utils.filter_color(copia, low, high)                
            img, centro_yellow  =  q3utils.center_of_mass_region(mask, 0, 300, mask.shape[1], mask.shape[0])  

            saida_bgr, m, h = q3utils.ajuste_linear_grafico_x_fy(mask)

            ang = math.atan(m)
            ang_deg = math.degrees(ang)

            q3utils.texto(saida_bgr, f"Angulo graus: {ang_deg}", (15,50))
            q3utils.texto(saida_bgr, f"Angulo rad: {ang}", (15,90))

            cv2.imshow("centro", img)
            cv2.imshow("angulo", saida_bgr)


        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

if __name__=="__main__":

    rospy.init_node("q3")

    topico_imagem = "/camera/image/compressed"
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    pose_sub = rospy.Subscriber('/odom', Odometry , mypose)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)

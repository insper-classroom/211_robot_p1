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

centro_caixa = (320, 240)
area_caixa = 0

## 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global centro_yellow
    global m
    global angle_yellow
    global centro_caixa
    global area_caixa

    ### 
    ## Vamos fazer o gabarito para a caixa azul, que era mais distante 

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

            angle_yellow = ang_deg

            q3utils.texto(saida_bgr, f"Angulo graus: {ang_deg}", (15,50))
            q3utils.texto(saida_bgr, f"Angulo rad: {ang}", (15,90))

            cv2.imshow("centro", img)
            cv2.imshow("angulo", saida_bgr)


            ## Achando o maior objeto azul 
            media, centro_frame, area = q3utils.identifica_cor(copia)
            
            area_caixa = area
            centro_caixa = media



        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)





if __name__=="__main__":

    rospy.init_node("q3")

    topico_imagem = "/camera/image/compressed"
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    cmd_vel = velocidade_saida

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    pose_sub = rospy.Subscriber('/odom', Odometry , mypose)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))         


    x = 0
    tol_centro = 10 # tolerancia de fuga do centro
    tol_ang = 15 # tolerancia do angulo
    area_ideal_caixa = 10000

    c_img = 320 # Centro da imagem

    v_slow = 0.2
    v_rapido = 0.5
    w_slow = 0.1 
    w_rapido = 0.6

    
    INICIAL= -1
    AVANCA = 0
    AVANCA_RAPIDO = 1
    ALINHA = 2
    AVANCA_PROXIMO = 3
    TERMINOU = 4

    state = INICIAL

    def inicial():
        # Ainda sem uma ação específica
        pass

    def avanca():
        vel = Twist(Vector3(v_slow,0,0), Vector3(0,0,0)) 
        cmd_vel.publish(vel) 

    def avanca_rapido():
        vel = Twist(Vector3(v_slow,0,0), Vector3(0,0,0))         
        cmd_vel.publish(v_rapido)

    def alinha():
        delta_x = c_img - centro_centro_yellow[x]
        max_delta = 150.0
        w = w_min + (delta_x/max_delta)*w_rapido
        vel = Twist(Vector3(v_slow,0,0), Vector3(0,0,w)) 
        cmd_vel.publish(vel)        

    def avanca_proximo():
        pass

    def terminou():
        zero = Twist(Vector3(0,0,0), Vector3(0,0,0))         
        cmd_vel.publish(zero)

    def dispatch():
        "Logica de determinar o proximo estado"
        if area_caixa >= area_ideal_caixa:
            state = TERMINOU
        elif c_img - tol_centro < centro_yellow[x] < c_img + tol_centro:
            state = AVANCA

            if   - tol_ang< angle_yellow  < tol_ang:  # para angulos centrados na vertical, regressao de x = f(y) como está feito
                state = AVANCA_RAPIDO
        else:
            state = ALINHA
        print("centro_yellow {} area caixa {:.2f} angle_yellow {:.3f}".format(centro_yellow, area_caixa, angle_yellow))
    acoes = {INICIAL:inicial, AVANCA: avanca, AVANCA_RAPIDO: avanca_rapido, ALINHA: alinha, AVANCA_PROXIMO: avanca_proximo , TERMINOU: terminou}


    while not rospy.is_shutdown():
        print("Estado: ", state)
        acoes[state]()  # executa a funcão que está no dicionário
        dispatch()    
        rospy.sleep(0.05)

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

## @@ ## --- Troque Aqui Entre as Cores --- ## @@ ##
global cor
# cor = 'Magenta'
cor = 'Amarelo'

armazenar_pose = [10000., 0.]

ranges = None
minv = 0
maxv = 10

bridge = CvBridge()

x = -1
y = -1
contador = 0
radianos = None
pula = 10
X = [0]
na_image = False

## Processamento de Imagem
def filtra_image(hsv, lower_blue, upper_blue):
    """Filtrar o range da cor. Aplica OPENING na imagem para remover FP"""
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((10,10),np.uint8))

    return mask

def encontrar_contornos(mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca os contornos encontrados
    """
    # RETR_EXTERNAL: Apenas Contornos Externos
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    return contornos

def crosshair(cv_image, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(cv_image,(x - size,y),(x + size,y),color,2)
    cv2.line(cv_image,(x,y - size),(x, y + size),color,2)
    return cv_image

def encontrar_centro_dos_contornos(cv_image, contornos):
    """Não mude ou renomeie esta função
        deve receber um contorno e retornar, respectivamente, a imagem com uma cruz no centro de cada segmento e o centro dele. formato: img, x, y
    """
    global X
    X = []
    Y = []
    area = []
    for contorno in contornos:
        M = cv2.moments(contorno)

        # Usando a expressão do centróide
        if M["m00"] != 0: 
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centro = (cX, cY)
            crosshair(cv_image, centro, 5, [255, 0, 0])
            X.append(cX)
            Y.append(cY)
            area.append(cv2.contourArea(contorno))
    return cv_image

##
def recebe_odometria(data):
    """
    Recebe a odometria do robo.
    """
    global x
    global y
    global contador
    global angulos
    global armazenar_pose

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]

    # Angulos tem os angulos absolutos em graus
    angulos = np.degrees(transformations.euler_from_quaternion(lista)) # Angulo esta no range [-180,180]

    ## Converter o Yaw para o range [0, 360] (Facilita Muito!)
    angulos[2] = (angulos[2] + 360) % 360

    # Angulos tem os angulos absolutos em radianos
    radianos =   transformations.euler_from_quaternion(lista)

    ## Se a distancia for menor da armazenada E
    ##  Se a caixa esta na imagem
    ##   Guarda a distancia e o Yaw do robo
    if minv < armazenar_pose[0] and na_image is True:
        armazenar_pose = [minv, angulos[2]]

    if contador % pula == 0:
        print("Distancia minv {:.2f} - angulo {:.2f}".format(minv,angulos[2]))
        print(armazenar_pose)
    contador = contador + 1

def scaneou(dado):
    global ranges
    global minv
    global maxv
    # A variável ranges vai conter as leituras do laser
    ranges = np.array(dado.ranges).round(decimals=2)
    # minv = dado.range_min 
    minv = ranges[0]
    maxv = dado.range_max
 
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global na_image
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        if cor == 'Amarelo':
            lower_blue = np.array([22,50,50])
            upper_blue = np.array([37,255,255])

            mask = filtra_image(hsv, lower_blue, upper_blue)
        
        if cor == 'Magenta':
            lower_blue = np.array([142,50,50])
            upper_blue = np.array([157,255,255])

            mask = filtra_image(hsv, lower_blue, upper_blue)

        ## Encontrar Contornos
        contornos = encontrar_contornos(mask)

        ## Indica Se a Caixa Esta na Imagem
        if not contornos:
            na_image = False
        else:
            na_image = True

            ## Encontrar Centro de Cada Peca
            cv_image = encontrar_centro_dos_contornos(cv_image, contornos)

        cv2.imshow("Camera", cv_image)
        # cv2.imshow("Camera", mask)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

w_vel = 0.6

if __name__=="__main__":

    ## Iniciar Maquina de Estados
    state = 0

    rospy.init_node("q4")

    topico_imagem = "/camera/image/compressed"

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    
    r = rospy.Rate(100) # Roda em ~100hz 
    while not rospy.is_shutdown():
        if state == 0:
            """Estado 0: Robo inicia a volta, passa para o estado 1"""
            start_time = rospy.Time.now()
            vel = Twist(Vector3(0,0,0), Vector3(0,0,w_vel))
            velocidade_saida.publish(vel)

            state = 1
        
        if state == 1:
            """Estado 1: Com movimento constante, robo verifica se passou o tempo e entao passa para o estado 2"""
            vel = Twist(Vector3(0,0,0), Vector3(0,0,w_vel))
            
            ## Tempo = Distancia / Velocidade
            if rospy.Time.now() - start_time >= rospy.Duration.from_sec(2.1 * math.pi / w_vel):
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

                state = 2

            velocidade_saida.publish(vel)

        if state == 2:
            """Estado 2: Robo verifica a posicao da caixa e direciona o movimento para ela, passa para o estado 3"""
            distancia_angulo = armazenar_pose[1] - angulos[2]
            print(distancia_angulo)

            if distancia_angulo < 0:
                w_vel = - w_vel

            distancia_angulo = np.radians(abs(distancia_angulo))
            start_time = rospy.Time.now()
            vel = Twist(Vector3(0,0,0), Vector3(0,0,w_vel))
            velocidade_saida.publish(vel)

            state = 3

        if state == 3:
            """Estado 3: Com movimento constante, robo verifica se passou o tempo e para na caixa, 
                passa para o estado 4 (estado neutro)"""
            vel = Twist(Vector3(0,0,0), Vector3(0,0,w_vel))

            if rospy.Time.now() - start_time >= rospy.Duration.from_sec(abs(distancia_angulo / w_vel)):
                state = 4

        if state == 4:
            """Estado 3: Com movimento constante, robo verifica se passou o tempo e para na caixa, 
                passa para o estado 4 (estado neutro)"""
            print("Adjusting")
            print(X)
            if X[0] > 338 and  X[0] < 342:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                
                state = 5
            elif X[0] > 240:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
            elif X[0] < 240:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
        
            velocidade_saida.publish(vel)

        r.sleep()
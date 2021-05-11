#!/usr/bin/env python

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
# from std_msgs.msg import String # For pub/sub
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage # Subscriber
from sensor_msgs.msg import Image # Publisher
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
import numpy as np

# # import os # Para teste
# # import time

# @@@@@@ Use this on target folder to make node executable: chmod +x [filename].py
# . ~/catkin_ws/devel/setup.bash

class image_converter:

    def __init__(self):
        ## Escolher a cor:
        self.cor = 'Magenta'
        # self.cor = 'Amarelo'

        self.area = [0]
        self.na_image = 0

        self.image_pub = rospy.Publisher("/target_image",Image, queue_size=1)
        self.pub = rospy.Publisher('/target', Pose)

        self.bridge = CvBridge()
        rospy.init_node('q4imagemodule', anonymous=True)

        topico_imagem = "/camera/image/compressed"
        self.image_sub = rospy.Subscriber(topico_imagem,CompressedImage,self.callback)

    def callback(self,imagem):
        """
        Converte a imagem do robo para uma imagem no OpenCV
        Filtra a caixa desejada, calcula o centro.

        Publica para dois topicos:



        :param data: sensor_msg/CompressedImage. Imagem obtido no topico.
        :return: cv_image. cv::Mat. Imagem no ambiente do OpenCV

        """
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        except CvBridgeError as e:
            print(e)

        # # @@@@@@ Start @@@@@@

        self.roda_todo_frame()
        self.monta_output()
        self.pub.publish(self.target_pose)
        # # @@@@@@ End @@@@@@

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8")) # Publica a imagem como sensor_msg/Image
        except CvBridgeError as e:
            print(e)

    def monta_output(self):
        self.target_pose = Pose()
        # self.target_pose.header.stamp = rospy.Time.now() # Para montar o header da menssagem
        # PS: No caso, Pose() nao tem header.

        self.target_pose.position.z = self.na_image

        if self.na_image == 1:
            self.target_pose.position.x = float(self.X[0])
            self.target_pose.position.y = float(self.Y[0])

    def roda_todo_frame(self):

        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        if self.cor == 'Amarelo':
            lower_blue = np.array([22,50,50])
            upper_blue = np.array([37,255,255])

            mask = self.filtra_image(hsv, lower_blue, upper_blue)
        
        if self.cor == 'Magenta':
            lower_blue = np.array([142,50,50])
            upper_blue = np.array([157,255,255])

            mask = self.filtra_image(hsv, lower_blue, upper_blue)

        ## Encontrar Contornos
        contornos = self.encontrar_contornos(mask)

        ## Indica Se a Caixa Esta na Imagem
        if not contornos:
            self.na_image = 0
        else:
            self.na_image = 1

            ## Encontrar Centro de Cada Peca
            self.encontrar_centro_dos_contornos(contornos)
    
    ## Processamento de Imagem

    def filtra_image(self,hsv, lower_blue, upper_blue):
        """Filtrar o range da cor. Aplica OPENING na imagem para remover FP"""
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((10,10),np.uint8))

        return mask

    def encontrar_contornos(self,mask):
        """Não mude ou renomeie esta função
            deve receber uma imagem preta e branca os contornos encontrados
        """
        # RETR_EXTERNAL: Apenas Contornos Externos
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(self.cv_image, contornos, -1, (0, 255, 0), 3)
        return contornos

    def crosshair(self, point, size, color):
        """ Desenha um crosshair centrado no point.
            point deve ser uma tupla (x,y)
            color é uma tupla R,G,B uint8
        """
        x,y = point
        cv2.line(self.cv_image,(x - size,y),(x + size,y),color,2)
        cv2.line(self.cv_image,(x,y - size),(x, y + size),color,2)

    def encontrar_centro_dos_contornos(self, contornos):
        """Não mude ou renomeie esta função
            deve receber um contorno e retornar, respectivamente, a imagem com uma cruz no centro de cada segmento e o centro dele. formato: img, x, y
        """
        self.X = []
        self.Y = []
        for contorno in contornos:
            M = cv2.moments(contorno)

            # Usando a expressão do centróide
            if M["m00"] != 0: 
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centro = (cX, cY)
                self.crosshair(centro, 5, [255, 0, 0])
                self.X.append(cX)
                self.Y.append(cY)
                self.area.append(cv2.contourArea(contorno))

##



def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
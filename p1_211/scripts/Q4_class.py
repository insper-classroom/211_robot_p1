#!/usr/bin/env python

from __future__ import print_function

import roslib
import sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import Odometry
from tf import transformations
import math

# # import os # Para teste
# # import time

# @@@@@@ Use this on target folder to make node executable: chmod +x [filename].py
# . ~/catkin_ws/devel/setup.bash

class SolveQ4:

    def __init__(self):
        self.X, self.Y, self.na_image = 0, 0, 0 # pos X, Y do bloco e se existe bloco da cor na imagem
        self.x, self.y = 0, 0 # pos x, y do robo
        self.contador = 0
        self.armazenar_pose = [10000., 0.]
        self.ranges = None
        self.minv = 0
        self.maxv = 10
        self.angulos = [0,0,0]

        rospy.init_node('q4', anonymous=True)
        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

        recebe_scan = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        ref_odometria = rospy.Subscriber("/odom", Odometry, self.recebe_odometria)
        ref_target = rospy.Subscriber("/target", Pose, self.recebe_target)

        self.state = '0'
        self.w_vel = 0.8
        self.state_machine = {
            '0': self.state_0,
            '1': self.state_1,
            '2': self.state_2,
            '3': self.state_3,
            '4': self.state_4,
            '5': print('done')
            }
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            print(self.state)
            self.state_machine[self.state]()
            print(self.state)
            r.sleep()

    def recebe_target(self, data):
        self.X = data.position.x
        self.Y = data.position.y
        self.na_image = data.position.z

    def recebe_odometria(self,data):
        """
        Recebe a odometria do robo.
        """
        global contador
        global angulos
        global armazenar_pose

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]

        # Angulos tem os angulos absolutos em graus
        self.angulos = np.degrees(transformations.euler_from_quaternion(lista)) # Angulo esta no range [-180,180]

        ## Converter o Yaw para o range [0, 360] (Facilita Muito!)
        self.angulos[2] = (self.angulos[2] + 360) % 360

        # # Angulos tem os angulos absolutos em radianos
        # radianos =   transformations.euler_from_quaternion(lista)

        ## Se a distancia for menor da armazenada E
        ##  Se a caixa esta na imagem
        ##   Guarda a distancia e o Yaw do robo
        if self.minv < self.armazenar_pose[0] and self.na_image == 1:
            self.armazenar_pose = [self.minv, self.angulos[2]]

    def scaneou(self,dado):
        # A variável ranges vai conter as leituras do laser
        self.ranges = np.array(dado.ranges).round(decimals=2)
        # minv = dado.range_min 
        self.minv = self.ranges[0]
        self.maxv = dado.range_max

    def state_0(self):
        """Estado 0: Robo inicia a volta, passa para o estado 1"""
        self.start_time = rospy.Time.now()
        vel = Twist(Vector3(0,0,0), Vector3(0,0,self.w_vel))
        self.velocidade_saida.publish(vel)

        self.state = '1'

    def state_1(self):
        """Estado 1: Com movimento constante, robo verifica se passou o tempo e entao passa para o estado 2"""
        vel = Twist(Vector3(0,0,0), Vector3(0,0,self.w_vel))
        
        ## Tempo = Distancia / Velocidade
        if rospy.Time.now() - self.start_time >= rospy.Duration.from_sec(2.1 * math.pi / self.w_vel):
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

            self.state = '2'

        self.velocidade_saida.publish(vel)
    
    def state_2(self):
        """Estado 2: Robo verifica a posicao da caixa e direciona o movimento para ela, passa para o estado 3"""
        self.distancia_angulo = self.armazenar_pose[1] - self.angulos[2]
        print(self.distancia_angulo)

        if self.distancia_angulo < 0:
            self.w_vel = - self.w_vel

        self.distancia_angulo = np.radians(abs(self.distancia_angulo))
        self.start_time = rospy.Time.now()
        vel = Twist(Vector3(0,0,0), Vector3(0,0,self.w_vel))
        self.velocidade_saida.publish(vel)

        self.state = '3'
    
    def state_3(self):
        """Estado 3: Com movimento constante, robo verifica se passou o tempo e para na caixa, 
            passa para o estado 4 """
        vel = Twist(Vector3(0,0,0), Vector3(0,0,self.w_vel))

        if rospy.Time.now() - self.start_time >= rospy.Duration.from_sec(abs(self.distancia_angulo / self.w_vel)):
            self.state = '4'
    
    def state_4(self):
        """Estado 4: Robo centraliza na caixa e passa para o estado 5 (estado neutro)"""
        print("Adjusting")

        if self.X > 338 and  self.X < 342:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            
            self.state = '5'
        elif self.X > 338:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
        elif self.X < 342:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
    
        self.velocidade_saida.publish(vel)



######
def main(args):
  ic = SolveQ4()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
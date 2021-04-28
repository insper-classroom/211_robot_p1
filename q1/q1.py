#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
files = "chess01.png chess02.png chess03.png chess04.png".split()

nd = " " # vazio


# Só teremos reis e peões e o tabuleiro é 5x5 somente
rei_preto = "BK" # black king
peao_preto = "BP"  # black pawn
rei_branco = "WK" # white king
peao_branco = "WP" # white pawn

def encontrar_contornos(mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca os contornos encontrados
    """
    # RETR_EXTERNAL: Apenas Contornos Externos
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    return contornos

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)

def encontrar_centro_dos_contornos(img, contornos):
    """Não mude ou renomeie esta função
        deve receber um contorno e retornar, respectivamente, a imagem com uma cruz no centro de cada segmento e o centro dele. formato: img, x, y
    """
    img_copia = img.copy()
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
            crosshair(img_copia, centro, 5, [0, 0, 255])
            X.append(cX)
            Y.append(cY)
            area.append(cv2.contourArea(contorno))

    return img_copia, X, Y, area

# Trabalhe nesta função
# Pode criar e chamar novas funções o quanto quiser

def peca_e_cor(img, x,y,area):
    if area > 2200: # Area do Rei:~2800 px
        if img[y][x+10][0] == 255: ## Deslocado a 10px
            return rei_branco
        else:
            return rei_preto
    else: # Area do Peao:~1900 px
        if img[y][x][0] == 255:
            return peao_branco
        else:
            return peao_preto

def processa(frame_bgr): 
    img = frame_bgr.copy()

    ## Converter Para Cinza
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ## Filtrar Cor
    output = cv2.inRange(gray,5,250)
    output = cv2.bitwise_not(output)
    output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, np.ones((10,10),np.uint8))

    ## Encontrar Contornos
    contornos = encontrar_contornos(output)

    ## Encontrar Centro de Cada Peca
    img, X, Y, area = encontrar_centro_dos_contornos(img, contornos)

    ## Gerar Tabuleiro
    board = [[nd,nd,nd,nd,nd], [nd,nd,nd,nd,nd], [nd,nd,nd,nd,nd], [nd,nd,nd,nd,nd], [nd,nd,nd,nd,nd]]    
    tabuleiro = board.copy()

    ## Montar Tabuleiro
    for i in range(len(X)):
        x = X[i]//93            # Tabuleiro: 465x465 --- Casa: 93x93
        y = Y[i]//93            # Divisão dupla para retornar um inteiro
        tabuleiro[y][x] = peca_e_cor(img,X[i],Y[i],area[i])

    return tabuleiro, img



if __name__ == "__main__":

    # Inicializa a leitura dos arquivos
    bgr = [cv2.imread(f) for f in files]
    
    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")

    for frame in bgr: 
        # Capture frame-by-frame

        # Our operations on the frame come here
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Você vai trabalhar na função processa!
        tabuleiro, imagem = processa(frame)

        print("Tabuleiro")
        print(tabuleiro)

        # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv2.imshow
        cv2.imshow('imagem processada', imagem)

        if cv2.waitKey(1500) & 0xFF == ord('q'):
            break




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


# Trabalhe nesta função
# Pode criar e chamar novas funções o quanto quiser
def processa(frame_bgr): 
    img = frame_bgr.copy()    
    # Lista vazia para ser tabuleiro
    board = [[nd,nd,nd,nd,nd], [nd,nd,nd,nd,nd], [nd,nd,nd,nd,nd], [nd,nd,nd,nd,nd], [nd,nd,nd,nd,nd]]    
    tabuleiro = board.copy()
    # Este código é só exemplo da lógica 
    # Use a notação linha x coluna
    tabuleiro[1][1] = "BK"
    tabuleiro[4][4] = "WK"
    tabuleiro[0][0] = "WP"
    tabuleiro[0][1] = "BP"
    # Seu código deve retornar o tabuleiro e uma imagem 
    # que demonstre alguma saída visual
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




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
video = "bandeiras_movie.mp4"

def encontrar_contornos(mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca os contornos encontrados
    """
    # RETR_EXTERNAL: Apenas Contornos Externos
    contornos, arvore = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    return contornos, arvore

def encontrar_caixa_do_contorno(contornos):
    """
    Encontra a "bounding box" de cada contorno.
    """
    box = []
    for contorno in contornos:
        (x,y,w,h) = cv2.boundingRect(contorno)
        box.append((x,y,w,h))

    return box
    
def escrever_textos(frame,box, color, text):
    """
    Escrever os pontos P1 e P2, a bandeira e desenhar um retangulo envolta da bandeira.
    """
    (x,y,w,h) = box
    
    # Retangulo
    cv2.rectangle(frame, (x,y), (x+w,y+h), color, 3)

    font = cv2.FONT_HERSHEY_SIMPLEX
    # Flag
    cv2.putText(frame, text, (x,y+5*h//4), font, 0.75, color, 2, cv2.LINE_AA)
    # P1
    cv2.putText(frame, 'P1 (%s,%s)'%(x,y), (x,y-5), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
    # P2
    cv2.putText(frame, 'P2 (%s,%s)'%(x+w,y+h), (x+w+5,y+h), font, 0.5, (255,255,255), 1, cv2.LINE_AA)
    

    return frame, ((x,y),(x+w,y+h))


######
corner_jp = ((10,10),(100,100))
corner_pl = ((5,5),(200,200))

# Responda dentro desta função. 
# Pode criar e chamar novas funções o quanto quiser
def encontra_japao_polonia_devolve_corners(bgr):
    frame = bgr.copy()

    ## Converter Para Cinza
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ## Filtrar Cor
    output = cv2.inRange(gray,200,255)
    # Para remover FP:
    output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, np.ones((10,10),np.uint8))

    ## Encontrar Contornos
    contornos, _ = encontrar_contornos(output)

    ## Ordenando eh Possivel Separar Apenas Japao e Polonia
    contornos = sorted(contornos,key=cv2.contourArea)[5:7]

    ## Encontrar Bounding Box
    box = encontrar_caixa_do_contorno(contornos)

    ## Japao
    frame, corner_jp = escrever_textos(frame, box[1], (255,0,0), 'Japao')

    ## Polonia
    (x,y,w,h) = box[0]
    # Dobrar a Altura do Retangulo Para Incluir o Retangulo Vermelho
    frame, corner_pl = escrever_textos(frame, (x,y,w,h*2), (0,255,0), 'Polonia')

    return frame, corner_jp, corner_pl

if __name__ == "__main__":

    # Inicializa a aquisição da webcam
    cap = cv2.VideoCapture(video)

    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret == False:
            #print("Codigo de retorno FALSO - problema para capturar o frame")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
            #sys.exit(0)

        # Our operations on the frame come here
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Programe só na função encontra_japao_devolve_corners. Pode criar funções se quiser
        saida, japao, polonia = encontra_japao_polonia_devolve_corners(frame)

        print("Corners x-y Japao")
        print(japao)
        print("Corners x-y Polonia")
        print(polonia)

        # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv2.imshow
        cv2.imshow('imagem', saida)

        if cv2.waitKey(15) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()



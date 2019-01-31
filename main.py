# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 20:09:45 2019

@author: victo
"""
import cv2
import math
import time
#Cargar Video
camara = cv2.VideoCapture("./video/BANDA_AGUA.mp4")
# Variables
fondo = None #Nos servirá para obtener el fondo
PosicionQ=[] #Posiciones de los Cuadros
PosicionQ2=[] #Posiciones de los Cuadros externos
PosicionQ3=[]
tamanoRecX = 100
tamanoRecY = 0
condicion = 0
vel_avance = 100 #velocidad de avance de la banda en m/s
longitudCam = 0.5 #longitud de la camara en m
#Reproduccion de video
tiempo_in = time.time()
tiempo_frame = 0
while True:
    #Obtiene el frame
    (grabbed, frame) = camara.read()
    #Finaliza el ciclo cuando termina el video
    if not grabbed:
        break
    K = longitudCam / frame.shape[1] #Constante de Relacion entre longitud de la imagen y del area m/pixel
    # Convertimos a escala de grises
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Aplicamos suavizado para eliminar ruido
    gris2 = cv2.GaussianBlur(gris, (21, 21), 0)
    # Será el primer frame que obtengamos
    if fondo is None:
        fondo = gris2
        continue
    # Calculo de la diferencia entre el fondo y el frame actual
    resta = cv2.absdiff(fondo, gris2)
    # Aplicamos un umbral
    umbral = cv2.threshold(resta, 25, 255, cv2.THRESH_BINARY)[1]
    # Dilatamos el umbral para tapar agujeros
    umbral = cv2.dilate(umbral, None, iterations=2)
    # Copiamos el umbral para detectar los contornos
    contornosimg = umbral.copy()
    # Buscamos contorno en la imagen
    im, contornos, hierarchy = cv2.findContours(contornosimg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # Recorremos todos los contornos encontrados
    for c in contornos:
        # Eliminamos los contornos más pequeños
        if cv2.contourArea(c) < 500:
            continue
        # Obtenemos el bounds del contorno, el rectángulo mayor que engloba al contorno
        (x, y, w, h) = cv2.boundingRect(c)
        #Almacena las pocisiones de los cuadros detectados
        PosicionQ = PosicionQ + [[x, y, w, h]]
    #Tamaño y posicion del rectangulo de deteccion
    tamanoRecY = frame.shape[0]
    PosicionX = int(frame.shape[1] / 2)
    PosicionY = int(frame.shape[0] / 2)
    xRecIn = PosicionX - int(tamanoRecX / 2)
    xRecF = PosicionX + int(tamanoRecX / 2)
    yRecIn = PosicionY - int(tamanoRecY / 2)
    yRecF = PosicionY + int(tamanoRecY / 2)
    #Extraccion de los rectangulos externos
    for e in range(0,len(PosicionQ)):
        for d in range(0,len(PosicionQ)):
            if d == e:
                continue
            if (PosicionQ[e][0] >= PosicionQ[d][0]) and ((PosicionQ[e][0] + PosicionQ[e][2]) <= (PosicionQ[d][0] + PosicionQ[d][2])) and (PosicionQ[e][1] >= PosicionQ[d][1]) and ((PosicionQ[e][1] + PosicionQ[e][3]) <= (PosicionQ[d][1] + PosicionQ[d][3])):
                condicion = 1
                break
            else:
                condicion = 0
        #se copian los rectangulos que son externos
        if condicion == 0:
            PosicionQ2 = PosicionQ2 + [PosicionQ[e]]
            centroX = PosicionQ[e][0] + int(PosicionQ[e][2]/2)
            centroY = PosicionQ[e][1] + int(PosicionQ[e][3]/2)
            if len(PosicionQ3) > 0:
                for j in PosicionQ3:
                    PuntoX = j[0] + int(j[2]/2)
                    PuntoY = j[1] + int(j[3]/2)
                    radioM = math.sqrt(pow(centroX - PuntoX,2)+pow(centroY - PuntoY,2))
                    print(radioM * K / tiempo_frame)
                    #####Continuar con la idea de saber cual cuadro corresponde con el frame siguiente
        condicion = 0
    for f in PosicionQ2:
        # Dibujamos el rectángulo del bounds
        centroX=f[0] + int(f[2]/2)
        centroY=f[1] + int(f[3]/2)
        cv2.rectangle(frame, (f[0], f[1]), (f[0] + f[2], f[1] + f[3]), (0, 0, 255), 2)
    PosicionQ3 = PosicionQ2
    PosicionQ = []
    PosicionQ2 = []
    condicion = 0
    tiempo_frame = time.time() - tiempo_in
    tiempo_in = time.time()
    #Visualiza el video
    cv2.imshow("Camara", frame)
    cv2.imshow("Resta", resta)

    # Capturamos una tecla para salir
    key = cv2.waitKey(1) & 0xFF
    # Si ha pulsado la letra s, salimos
    if key == ord("s"):
        break
camara.release()
cv2.destroyAllWindows()
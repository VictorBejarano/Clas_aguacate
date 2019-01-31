# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 20:09:45 2019

@author: victo
"""
import cv2
#Cargar Video
camara = cv2.VideoCapture("./video/BANDA_AGUA.mp4")
# Variables
fondo = None #Nos servirá para obtener el fondo
PosicionQ=[] #Posiciones de los Cuadros
tamanoRecX = 100
tamanoRecY = 0
#Reproduccion de video
while True:
    #Obtiene el frame
    (grabbed, frame) = camara.read()
    #Finaliza el ciclo cuando termina el video
    if not grabbed:
        break
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
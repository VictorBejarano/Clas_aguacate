# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 20:09:45 2019

@author: victo
"""
import cv2
#Cargar Video
camara = cv2.VideoCapture("./video/BANDA_AGUA.mp4")

#Reproduccion de video
while True:
    #Obtiene el frame
    (grabbed, frame) = camara.read()
    #Finaliza el ciclo cuando termina el video
    if not grabbed:
        break
    #Visualiza el video
    cv2.imshow("Camara", frame)
    # Capturamos una tecla para salir
    key = cv2.waitKey(1) & 0xFF
    # Si ha pulsado la letra s, salimos
    if key == ord("s"):
        break
camara.release()
cv2.destroyAllWindows()
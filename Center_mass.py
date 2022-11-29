import cv2
import numpy as np
import imutils
import os
import colorsys
import threading

def agregar_circulos(color_RGB, imagen):
    hsv_image = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
    color_hsv = cv2.cvtColor(np.uint8([[color_RGB]]),cv2.COLOR_BGR2HSV)  #Color en HSV
    LowerColor = np.array([color_hsv[0][0][0]-10, 100, 100])#np.array([color_hsv[0][0][0]-10, 100, 100])
    UpperColor = np.array([color_hsv[0][0][0]+10, 255, 255])
    ColorBackMask = cv2.inRange(hsv_image, LowerColor, UpperColor)
    ColorBackBlur = cv2.GaussianBlur(ColorBackMask,(31,31),0)
    ColorBackRet, ColorBackOTSUMask = cv2.threshold(ColorBackBlur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    ColorBackRes = cv2.bitwise_and(imagen,imagen, mask= ColorBackOTSUMask)
    res = ColorBackRes

    ColorBackMoments = cv2.moments(ColorBackOTSUMask, 1)
    m00_ColorBack = int(ColorBackMoments['m00'])
    m01_ColorBack = int(ColorBackMoments['m01'])
    m10_ColorBack = int(ColorBackMoments['m10'])
    if m00_ColorBack == 0:
        m00_ColorBack = 0.01

    x_punto = int(m10_ColorBack/m00_ColorBack)
    y_punto = int(m01_ColorBack/m00_ColorBack)
    return res, (x_punto, y_punto)




class Centro_masa:
    def __init__(self) -> None:
        self.cap = cv2.VideoCapture(1) #Cambiar a 1 para usar camara usar 0 para computador
        self.mostrando_imagen = True
        self.agregar_colores()
        self.centro_x = 640//2
        pass


    def mostrar_imagen(self):
        while self.mostrando_imagen:
            ret, frame = self.cap.read()
            restos = []
            for color in self.colores:
                nuevo_color = color*255
                rest, coordenadas = agregar_circulos(nuevo_color, frame)
                color_lista = [int(nuevo_color[0]), int(nuevo_color[1]), int(nuevo_color[2])]
                cv2.circle(frame, coordenadas, 20, color_lista, -1)
                restos.append(rest)

            k = cv2.waitKey(1)
            if k == 27:  #terminar con esc
                break



            resto = sum(restos) #Ver centros de masa
            cv2.imshow('frame', frame)
            cv2.imshow('res', resto)
            #print(self.calcular_error())

            if not (self.cap.isOpened()):
                self.mostrando_imagen = False

    def agregar_colores(self):
        amarillo = np.array([0, 1, 1]) #180
        magenta = np.array([1, 0, 1]) #300
        verde = np.array([0, 1, 0]) #120
        rojo = np.array([0, 0, 1]) #240
        azul = np.array([1, 0, 0]) #0
        cyan = np.array([1, 1, 0]) #60
        self.colores = [] 
        #Aprovados
        #self.colores.append(magenta) #No hay falsos
        self.colores.append(verde) #Bien
        #self.colores.append(azul) #
        #self.colores.append(cyan) #

        #Reprobados
        #self.colores.append(rojo) #Lleno de falsos
        #self.colores.append(amarillo) #Se confunde con cafe

    def calcular_error(self):
        if (self.cap.isOpened()):
            ret, frame = self.cap.read()
            for color in self.colores:
                nuevo_color = color*255
                rest, coordenadas = agregar_circulos(nuevo_color, frame)
                respuesta = -self.centro_x + coordenadas[0]
                if coordenadas[0]==0:
                    respuesta = 0
                #print(respuesta)
                return respuesta



centro = Centro_masa()


#centro.mostrar_imagen()
import cv2
import numpy
import imutils
import os


cap = cv2.VideoCapture(0) #Cambiar a 1 para usar camara usar 0 para computador
x1, y1 = 190, 60
x2, y2 = 400, 450

#Codigo de fundamentos
LowerColorError = numpy.array([-10,-35,-35])
UpperColorError = numpy.array([10,35,35])


a = 0

def agregar_circulos(color_RGB, imagen):
    
    
    hsv_image = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
    color_hsv = cv2.cvtColor(numpy.uint8([[color]]),cv2.COLOR_BGR2HSV)  #Color en HSV
    LowerColor = numpy.array([color_hsv[0][0][0]-10, 100, 100])
    UpperColor = numpy.array([color_hsv[0][0][0]+10, 255, 255])
    ColorBackMask = cv2.inRange(hsv_image, LowerColor, UpperColor)
    ColorBackBlur = cv2.GaussianBlur(ColorBackMask,(31,31),0)
    ColorBackRet, ColorBackOTSUMask = cv2.threshold(ColorBackBlur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    ColorBackRes = cv2.bitwise_and(frame,frame, mask= ColorBackOTSUMask)
    res = ColorBackRes

    ColorBackMoments = cv2.moments(ColorBackOTSUMask, 1)
    m00_ColorBack = int(ColorBackMoments['m00'])
    m01_ColorBack = int(ColorBackMoments['m01'])
    m10_ColorBack = int(ColorBackMoments['m10'])
    if m00_ColorBack == 0:
        m00_ColorBack = 0.01

    x_punto = int(m10_ColorBack/m00_ColorBack)
    y_punto = int(m01_ColorBack/m00_ColorBack)

    #cv2.circle(frame, (x_punto, y_punto), 20, [color[0], color[1], color[2]], -1)
    return res, (x_punto, y_punto)


def RGB_to_BGR(color_entra):
    color_entra = color_entra*255
    return [color_entra[2], color_entra[1], color_entra[0]]


res = 1
while (cap.isOpened()):
    ret, frame = cap.read()

    cyan = numpy.array([0, 1, 1])
    magenta = numpy.array([1, 0, 1])
    verde = numpy.array([0, 1, 0])
    azul = numpy.array([0, 0, 1])
    rojo = numpy.array([1, 0, 0])
    amarillo = numpy.array([1, 1, 0])
    colores = []
    colores.append(cyan) #Se confunde con cafe
    colores.append(magenta) #No hay falsos
    colores.append(verde) #Bien
    colores.append(azul)
    colores.append(rojo) #Se confunde con azul
    restos = []
    for color in colores:
        nuevo_color = RGB_to_BGR(color)
        rest, coordenadas = agregar_circulos(nuevo_color, frame)
        color_lista = [int(nuevo_color[0]), int(nuevo_color[1]), int(nuevo_color[2])]
        cv2.circle(frame, coordenadas, 20, color_lista, -1)
        restos.append(rest)

    resto = sum(restos)
    k = cv2.waitKey(1)
    if k == 27:  #terminar con esc
        break

    cv2.imshow('frame', frame)
    cv2.imshow('res', resto)

#from curses import baudrate
import serial
import time
import serial.tools.list_ports as port_list
from threading import Timer
import cv2
import numpy as np
import imutils
import os
import colorsys
import threading

print(cv2.__version__)

'''ports = list(port_list.comports())
for p in ports:
    print (p)'''


class Sensor_profundidad:
    def __init__(self, com) -> None:
        self.conectar(com)
        pass

    def conectar(self, puerto):
        baud = 115200
        self.sensor = serial.Serial(port = puerto, baudrate = baud)

    def leer(self):
        leyendo = True
        self.sensor.flushInput()
        self.sensor.flushOutput()
        palabra = b""
        #print("reading")
        while leyendo:
            ans = self.sensor.read()
            palabra += ans
            if ans == b'\n':
                leyendo = False
        respuesta = self.decodificar(palabra)
        print("reading", respuesta)
        return respuesta


    def decodificar(self, codificado):
        #print(codificado)
        try:
            word = codificado.decode("ascii")
            cuenta = "saltar"
            for letra in range(len(word)):
                if word[letra]=='c':
                    cuenta = letra
            distancia = ''
            if cuenta != 'saltar':
                for numero in range(5):
                    #print(word[cuenta-numero])
                    if word[cuenta-numero].isnumeric():
                        distancia += (word[cuenta-numero])
                d1 = ''.join(reversed(distancia))
                #print("d1", d1)
                return d1
            else:
                print('salto', word)
                return cuenta


        except:
            print("No pudo decodificar", codificado)
            return "saltar"

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
    encontrado = True

    ColorBackMoments = cv2.moments(ColorBackOTSUMask, 1)
    m00_ColorBack = int(ColorBackMoments['m00'])
    m01_ColorBack = int(ColorBackMoments['m01'])
    m10_ColorBack = int(ColorBackMoments['m10'])
    if m00_ColorBack == 0:
        m00_ColorBack = 0.01
        encontrado = False

    x_punto = int(m10_ColorBack/m00_ColorBack)
    y_punto = int(m01_ColorBack/m00_ColorBack)
    return res, (x_punto, y_punto), encontrado


def contornos(imagen):
    gray = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 10, 150)
    canny = cv2.dilate(canny, None, iterations=1)
    canny = cv2.erode(canny, None, iterations=1)


    cnts,_ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #imagenHSV = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)

    contornos = []
    imAux = np.zeros(imagen.shape[:2], dtype="uint8")
    for c in cnts:
        imAux = cv2.drawContours(imAux, [c], -1, 255, -1)
        #maskHSV = cv2.bitwise_and(imagen, imagenHSV, mask= imAux)
        contornos.append(imAux)
    salida = sum(contornos)
    procesar = cv2.bitwise_and(imagen, imagen, mask= salida)
    #cv2.imshow('asdfg',salida)
    cv2.imshow('procesar',procesar)
    return procesar

class Centro_masa:
    def __init__(self) -> None:
        self.cap = cv2.VideoCapture(0) #Cambiar a 1 para usar camara usar 0 para computador
        self.mostrando_imagen = True
        self.agregar_colores()
        self.centro_x = 640//2
        self.y_max = 640 #500 #340
        self.y_min = 200 #250 #100

        pass



    def mostrar_imagen(self):
        print(self.centro_x)
        while self.mostrando_imagen:
            ret, frame = self.cap.read()
            restos = []

            frame = frame[:][self.y_min:self.y_max][:]
            #con_filtro = contornos(frame)
            for color in self.colores:
                nuevo_color = color*255
                rest, coordenadas, finded = agregar_circulos(nuevo_color, frame)
                color_lista = [int(nuevo_color[0]), int(nuevo_color[1]), int(nuevo_color[2])]
                #cv2.rectangle(frame, [0, self.y_max], [640, 0], [0, 0, 0], -1)
                cv2.circle(frame, [coordenadas[0], coordenadas[1]], 20, color_lista, -1)
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
        #Aprobados
        #self.colores.append(magenta) #No hay falsos
        #self.colores.append(verde) #Bien
        #self.colores.append(azul) #
        self.colores.append(cyan) #

        #Reprobados
        #self.colores.append(rojo) #Lleno de falsos
        #self.colores.append(amarillo) #Se confunde con cafe

    def calcular_error(self):
        if (self.cap.isOpened()):
            ret, frame = self.cap.read()
            frame = frame[:][self.y_min:self.y_max][:]
            #con_filtro = contornos(frame)
            
            for color in self.colores:
                nuevo_color = color*255
                rest, coordenadas, finded = agregar_circulos(nuevo_color, frame)
                respuesta = -self.centro_x + coordenadas[0]
                if coordenadas[0]==0:
                    respuesta = 0
                #print(respuesta)
                return respuesta


#centro = Centro_masa()

#for i in range(1000):
#    print(centro.calcular_error())
#    time.sleep(0.1)

#centro.mostrar_imagen()



class Control_angulo():
    def __init__(self, kp, ki, kd) -> None:
        self.kp_angulo = kp
        self.ki_angulo = ki
        self.kd_angulo = kd
        self.errores_angulo = [0, 0, 0]
        self.salida_anterior_angulo = 0
        self.tiempo = time.time()
        #self.periodo = 5
        self.dt = 0.1
        self.sensor = Centro_masa()
        pass

    def controlar_angulo(self):
        if (time.time()-self.tiempo) > self.dt:
            #print('f')
            error = self.sensor.calcular_error()
            
            self.errores_angulo[2] = self.errores_angulo[1]
            self.errores_angulo[1] = self.errores_angulo[0]
            self.errores_angulo[0] = error
            salida = self.salida_anterior_angulo + (self.kp_angulo + self.dt*self.ki_angulo + self.kd_angulo/self.dt)*error + (-self.kp_angulo + -2*self.kd_angulo/self.dt)*self.errores_angulo[1] + (self.kd_angulo/self.dt)*self.errores_angulo[2]
            if abs(salida)<10:
                salida = 0
            
            self.salida_anterior_angulo = salida
            self.tiempo = time.time()
            print("salida", salida)
            return salida
        return self.salida_anterior_angulo



ports = list(port_list.comports())
for p in ports:
    print (p)




class Motor:
    def __init__(self, port) -> None:
        self.port = port
        self.baudrate = 9600
        self.conectar()
        self.encender()
        

    def conectar(self):
        self.motor = serial.Serial(port = self.port, baudrate = self.baudrate)
        if self.motor.isOpen():
	        print("Conectado")
    
    def encender(self):
        self.motor.write(b'EN\n')
        #self.leer()
        print(self.port + " prendido")
    
    def mandar_velocidad(self, velocidad):
        try:
            int(velocidad)
            self.velocidad_actual = velocidad
            self.motor.write(bytes("V"+str(velocidad)+"\n", 'utf-8'))
            #self.leer()
        except:
            print("No es un numero")

    def apagar(self):
        self.motor.write(b'DI\n')
        #self.leer()

    def leer(self):
        leyendo = True
        palabra = b""
        while leyendo:
            ans = self.motor.read()
            palabra += ans
            if ans == b'\n':
                leyendo = False
        #print(palabra.decode('utf-8'))
        if palabra == b'OK\r\n':
            pass
        else:
            self.nueva_posicion = int(palabra)




class Controlador():
    def __init__(self, puerto1, puerto2, puerto3, Kp, Ki, Kd) -> None:
        self.motor_derecho = Motor(puerto1)
        self.motor_izquierdo = Motor(puerto2)
        self.limite_velocidad = 2000
        self.sensor_profundidad = Sensor_profundidad(puerto3)
        #self.ser = serial.Serial(port=puerto3, baudrate=19200, timeout=1)
        #self.ser.reset_input_buffer()
        self.controlador_angulo = Control_angulo(Kp, Ki, Kd)
        self.motor_derecho.mandar_velocidad(0)
        self.motor_izquierdo.mandar_velocidad(0)    
        self.commandos()
        pass


    def output(self):
        out = self.kp * self.list[-1] + self.integral * self.ki + self.kd * self.derivate
        if out < -self.limite_velocidad:
            out = -self.limite_velocidad
        if out > self.limite_velocidad:
            out = self.limite_velocidad
        return out

    def send_vel(self, vel):
        self.diferencia = 0
        if self.comando == 0:
            self.diferencia = 0
        self.motor_derecho.mandar_velocidad(-(vel+self.diferencia))
        self.motor_izquierdo.mandar_velocidad(vel)

    def avanzar_lento(self, vel):
        delta = int(vel/10)
        for i in range(10):
            self.send_vel(i*delta)
            time.sleep(0.2)
        self.send_vel(vel)

    def send_ang_speed(self, ang_speed):
        self.motor_derecho.mandar_velocidad(ang_speed)
        self.motor_izquierdo.mandar_velocidad(ang_speed)

    def send_ang_speed_vel(self, vel, ang_speed):
       velo = vel - ang_speed 
       self.motor_derecho.mandar_velocidad(-velo)
       self.motor_izquierdo.mandar_velocidad(vel)
       print(vel, ang_speed)




    def giro(self, sentido):
        #izquierda = 0
        #derecha = 1
        hora_inicio = time.time()
        proporcion = 1.82
        velocidad = 800
        tiempo_vuelta = 3
        tiempo = time.time() - hora_inicio 
        while tiempo < 20:  #revisar tiempo
            if sentido == 0:
                self.motor_derecho.mandar_velocidad(-velocidad/proporcion)
                self.motor_izquierdo.mandar_velocidad(velocidad)
            elif sentido == 1:
                self.motor_derecho.mandar_velocidad(-velocidad)
                self.motor_izquierdo.mandar_velocidad(velocidad/proporcion)
            print(tiempo)
            tiempo = time.time() - hora_inicio 
        self.motor_derecho.mandar_velocidad(0)
        self.motor_izquierdo.mandar_velocidad(0)
        
        
    def empezar_circuito(self):
        circuito_act = True
        cuenta = 0
        cantidad_vueltas = 3
        sensor_profundidad_conectado = True
        dist = 300
        time.sleep(1)
        print("Partio")
        while circuito_act: # Se convierte en falso cuando se termina circuito
            recta = True
            #tiempo_inicial = time.now()
            partiendo = False
            while partiendo and sensor_profundidad_conectado:
                num = self.sensor_profundidad.leer()
                print('Partiendo')
                if num.isnumeric():
                    if int(num)>dist:
                        partiendo = False
            while recta: #Se convierte en falso cuando es necesario hacer la curva
                self.linea_recta()
                #self.send_vel(500)
                print('Llendo recto')
                ##self.motor_derecho.mandar_velocidad(1200)
                #self.motor_izquierdo.mandar_velocidad(1200)
                if sensor_profundidad_conectado:
                    numero = self.sensor_profundidad.leer()
                else:
                    numero = 'sensor_profundidad_conectado'
                    #print('Termino')
                if numero.isnumeric():
                    print(numero)
                    if int(numero)<dist:
                        a = 1
                        recta = False
                        print("VUELTA!!!")
            self.giro(cuenta%2) #Revisar condicion para sentido
            rotando = False #True
            while rotando and sensor_profundidad_conectado:
                num = self.sensor_profundidad.leer()
                if num.isnumeric():
                    print('rotanado')
                    if int(num)>dist:
                        rotando = False
            cuenta += 1
            if cuenta == cantidad_vueltas: ##Agregarle mas para mas vueltas
                circuito_act = False #
                #self.sensor_profundidad.sensor.flushInput()
                break


    def tirar_agua(self, cantidad):
        self.ser.write(bytes(str(cantidad)+"\n", 'utf-8'))

    def leer_distancia(self):
        while True:
            print(self.sensor_profundidad.leer())


    def linea_recta(self):
        vel_motor = 1200
        #time.sleep(0.1)
        out = -self.controlador_angulo.controlar_angulo()
        salida_max = abs(out) + abs(vel_motor)
        if salida_max>self.limite_velocidad:
            print("Supero max")
            if out>0:
                out = out-salida_max-self.limite_velocidad
            else:
                out = -out+salida_max+self.limite_velocidad
        '''if out < -self.limite_velocidad:
            out = -self.limite_velocidad
        if out > self.limite_velocidad:
            out = self.limite_velocidad'''
        #print("our", out)
        self.send_ang_speed_vel(vel_motor, out)
        '''for i in range(5000):
            out = -self.controlador_angulo.controlar_angulo()
            if out < -self.limite_velocidad:
                out = -self.limite_velocidad
            if out > self.limite_velocidad:
                out = self.limite_velocidad
            #print(out)
            self.send_ang_speed_vel(500, out)
        self.send_vel( 0)'''
            

    def turn_off(self):
        self.motor_derecho.apagar()
        self.motor_izquierdo.apagar()

    def commandos(self):
        funcionando = True
        while funcionando:
            self.comando = input()
            if (self.comando == 'apagar') or (self.comando == '1'):
                funcionando = False
            elif (self.comando == '0'):
                self.send_vel(0)
                print("La velocidad es 0")
            elif self.comando == 'ava':
                self.send_vel(int(input("Velocidad lineal: ")))
            elif self.comando == 'ava y rot':
                self.send_ang_speed_vel(int(input("Velocidad lineal: ")), int(input("Velocidad angular: ")))
            elif self.comando == "rot":
                self.send_ang_speed(int(input("Velocidad angular: ")))
            elif self.comando == "c":
                self.empezar_circuito()
            elif self.comando == "l":
                while True:
                     self.linea_recta()
            elif self.comando == "p":
                vel = int(input("Velocidad lineal: "))
                for i in range(100):
                    self.send_vel(vel)
            elif self.comando == "g":
                self.avanzar_lento(800)
                self.giro(1)
            elif self.comando == 'tirar':
                self.tirar_agua(int(input("Cantidad agua ")))
            elif self.comando == 'leer':
                self.leer_distancia()
            elif self.comando == 'ace':
                self.avanzar_lento(int(input("Velocidad lineal: ")))
            else:
                print(self.comando + "no es valido")


        self.turn_off()
        print("Se apago")

    

control = Controlador('/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyACM0', 1, 0, 0.05)
#control = Controlador('COM7','COM4',"COM11", 2, 0, 0)
'''#control = Controlador('/dev/ttyUSB0','/dev/ttyUSB1', 1, 2, 3)

bj
#motor = Motor('COM6')
corriendo = True
while corriendo:
    decition = input()
    if decition == '1':
        corriendo = False
    elif decition == '2':
        print('Avanzar')
        motor.mandar_velocidad(int(input("Velocidad lineal: ")))'''



# medir la distancia avanzada usando tiempo
# medir el angulo rotado usando tiempo


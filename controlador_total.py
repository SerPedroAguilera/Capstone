#from curses import baudrate
import serial
import time
import serial.tools.list_ports as port_list
from control_pid_angulo import Control_angulo
from threading import Timer
#from sensor import Sensor_profundidad


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
    def __init__(self, puerto1, puerto2, Kp, Ki, Kd) -> None:
        self.motor_derecho = Motor(puerto1)
        self.motor_izquierdo = Motor(puerto2)
        self.limite_velocidad = 2000
        #self.sensor_profundidad = Sensor_profundidad()
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

    def send_ang_speed(self, ang_speed):
        self.motor_derecho.mandar_velocidad(ang_speed)
        self.motor_izquierdo.mandar_velocidad(ang_speed)

    def send_ang_speed_vel(self, vel, ang_speed):
        if ang_speed>0:
            print('giro derecha')
            print("Motor derecho:", -vel, "Motor izquierdo:", (abs((vel+ang_speed))))
            self.motor_derecho.mandar_velocidad(-abs(vel))
            self.motor_izquierdo.mandar_velocidad(abs((vel+ang_speed)))
        elif ang_speed<0:
            print('giro izquierda')
            print("Motor derecho:", -abs(vel+ang_speed), "Motor izquierdo:", vel)
            self.motor_derecho.mandar_velocidad(-abs(vel-ang_speed))
            self.motor_izquierdo.mandar_velocidad(abs(vel))
        else:
            print("Centrado")
            self.motor_derecho.mandar_velocidad(-vel)
            self.motor_izquierdo.mandar_velocidad(vel)
        ##self.motor_derecho.mandar_velocidad(abs(vel+ang_speed))
        #self.motor_izquierdo.mandar_velocidad(-(abs((vel-ang_speed))))



    def giro(self, sentido):
        #izquierda = 0
        #derecha = 1
        hora_inicio = time.time()
        proporcion = 1.82
        velocidad = 800
        tiempo_vuelta = 3
        tiempo = time.time() - hora_inicio 
        while tiempo < 10.7:  #revisar tiempo
            if sentido == 0:
                self.motor_derecho.mandar_velocidad(-velocidad)
                self.motor_izquierdo.mandar_velocidad(proporcion*velocidad)
            elif sentido == 1:
                self.motor_derecho.mandar_velocidad(-velocidad*proporcion)
                self.motor_izquierdo.mandar_velocidad(velocidad)
            print(tiempo)
            tiempo = time.time() - hora_inicio 
        self.motor_derecho.mandar_velocidad(0)
        self.motor_izquierdo.mandar_velocidad(0)
        
        
    def empezar_circuito(self):
        circuito_act = True
        cuenta = 0
        sensor_profundidad_conectado = False
        while circuito_act: # Se convierte en falso cuando se termina circuito
            recta = True
            #tiempo_inicial = time.now()
            partiendo = True
            while partiendo and sensor_profundidad_conectado:
                num = self.sensor_profundidad.leer()
                print('Partiendo')
                if num.isnumeric():
                    if int(num)>100:
                        partiendo = False
            while recta: #Se convierte en falso cuando es necesario hacer la curva
                self.linea_recta()
                #self.send_vel(1200)
                #print('Llendo recto')
                ##self.motor_derecho.mandar_velocidad(1200)
                #self.motor_izquierdo.mandar_velocidad(1200)
                if sensor_profundidad_conectado:
                    numero = self.sensor_profundidad.leer()
                else:
                    numero = 'sensor_profundidad_conectado'
                    #print('Termino')
                if numero.isnumeric():
                    if int(numero)<100:
                        a = 1
                        recta = False
                        print("VUELTA!!!")
            self.giro(1) #Revisar condicion para sentido
            rotando = True
            while rotando and sensor_profundidad_conectado:
                num = self.sensor_profundidad.leer()
                if num.isnumeric():
                    if int(num)>100:
                        rotando = False
            cuenta += 1
            if cuenta == 1: ##Agregarle mas para mas vueltas
                circuito_act = False #

            
            

    def linea_recta(self):
        vel_motor = 1200
        time.sleep(0.1)
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
                self.linea_recta()
            elif self.comando == "p":
                vel = int(input("Velocidad lineal: "))
                for i in range(100):
                    self.send_vel(vel)
            elif self.comando == "g":
                self.giro(1)
            else:
                print(self.comando + "no es valido")


        self.turn_off()
        print("Se apago")

    


control = Controlador('COM6','COM7', 2, 0, 0)
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


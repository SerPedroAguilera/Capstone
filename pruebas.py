#from curses import baudrate
import serial
import time
import serial.tools.list_ports as port_list
#from simple_pid import PID
from threading import Timer
import matplotlib.pyplot as plt


ports = list(port_list.comports())
for p in ports:
    print (p)

class PIDs:
    def __init__(self, kp, ki, kd) -> None:
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.limite_sup = 2000
        self.limite_inf = -2000
        self.errores = [0, 0, 0, 0, 0]
        
        pass

    def tunings(self, a, b, c):
        self.KP = a
        self.KI = b
        self.KD = c

    def output_limits(self, a, b):
        self.limite_inf = a
        self.limite_sup = b

    def calcular_voltaje(self, error, tiempo_pasado):
        self.errores.pop(0)
        self.errores +=  [error]
        mup = self.KP*error
        mui = self.KI * tiempo_pasado * sum(self.errores)
        mud = self.KD * (error - self.errores[3])/tiempo_pasado
        print("MUP", mup)
        if mup+mui+mud>self.limite_sup:
            return (self.limite_sup)
        elif mup+mui+mud<self.limite_inf:
            return (self.limite_inf)
        else:
            return (mup+mui+mud)


class Motor:
    def __init__(self, port) -> None:
        self.port = port
        self.baudrate = 9600
        self.kp = 15
        self.ki = 0
        self.kd = 0
        self.control_pid = PIDs(self.kp, self.ki, self.kd)
        self.control_pid.output_limits = (-2000, 2000)
        self.conectar()
        self.encender()
        self.posiciones = []

        self.velocidad_deseada = 10
        self.tiempo_anterior = time.time()
        self.errores = []
        self.voltaje_anterior = 0

        

    def conectar(self):
        self.motor = serial.Serial(port = self.port, baudrate = self.baudrate)
        if self.motor.isOpen():
	        print("Conectado")
    
    def encender(self):
        self.motor.write(b'EN\n')
        self.leer()
        print(self.port + " prendido")
        self.mandar_velocidad(0)
    
    def mandar_velocidad(self, velocidad):
        try:
            int(velocidad)
            self.velocidad_actual = velocidad
            self.motor.write(bytes("V"+str(velocidad)+"\n", 'utf-8'))
            self.leer()
        except:
            print("No es un numero")

    def apagar(self):
        self.motor.write(b'DI\n')
        self.leer()

    def leer(self):
        leyendo = True
        palabra = b""
        while leyendo:
            ans = self.motor.read()
            palabra += ans
            if ans == b'\n':
                leyendo = False
        print(palabra.decode('utf-8'))
        if palabra == b'OK\r\n':
            pass
        else:
            self.posiciones.append(int(palabra))


    def actualizar_potencia(self):
        self.motor.write(bytes("POS"+"\n", 'utf-8'))
        self.leer()
        diferencia_tiempo = time.time()-self.tiempo_anterior
        self.tiempo_anterior = time.time()
        if len(self.posiciones)!=1 and diferencia_tiempo<5:
            velocidad_actual_RPM = ((self.posiciones[-1]-self.posiciones[len(self.posiciones)-2])/(diferencia_tiempo))
        else:
            velocidad_actual_RPM = 0
        radio = 0.2
        conversion = (2*3.1415*radio)/700
        self.velocidad_actual = velocidad_actual_RPM*conversion
        print("RPM",  velocidad_actual_RPM)
        print("Tiempo", diferencia_tiempo)
        print("Velocidad deseada", self.velocidad_deseada, 'Velocidad actual', self.velocidad_actual)
        
        error = self.velocidad_deseada-self.velocidad_actual
        print(error)
        self.errores.append(error)
        voltaje_nuevo = self.control_pid.calcular_voltaje(error, diferencia_tiempo)
        #voltaje_nuevo = 25 * error
        self.mandar_velocidad(voltaje_nuevo + self.velocidad_deseada)#self.voltaje_anterior)
        print('Voltaje_anterior', self.voltaje_anterior, 'voltaje_nuevo', voltaje_nuevo)
        self.voltaje_anterior = voltaje_nuevo
        
        self.timer_potencia = Timer(0.5, self.actualizar_potencia)
        self.timer_potencia.start()


    def detener_ciclo(self):
        self.timer_potencia.cancel()

    def definir_pid(self):
        self.control_pid.KP = self.kp
        self.control_pid.KI = self.ki
        self.control_pid.KD = self.kd

    def graficar(self):
        tiempo = range(0, len(self.errores))
        print(self.velocidad_deseada, self.velocidad_actual)
        plt.plot(tiempo, self.errores)
        plt.title('Errores historicos')
        plt.text(0, 0,f"Kp: {self.kp} Ki: {self.ki} Kd:{self.kd}")
        plt.show()

    


        

motor = Motor('COM6')
corriendo = True

while corriendo:
    decition = input()
    if decition == '1': #Terminar codigo
        corriendo = False
    elif decition == '2': #Mandar velocidad
        print('Avanzar')
        motor.mandar_velocidad(int(input("Velocidad lineal: ")))
    elif decition == '3': #Detener timer
        motor.detener_ciclo()
        motor.mandar_velocidad(0)
    elif decition == '4': #Definir pid
        motor.kp = float(input("Kp: "))
        motor.ki = float(input("Ki: "))
        motor.kd = float(input("Kd: "))
        motor.definir_pid()
    elif decition == '5': #Hace andar controlador
        print('Controlador funcionando...')
        motor.actualizar_potencia()
    elif decition == '6': #Grafica error
        print('Graficando...')
        motor.graficar()
        motor.errores = []
    elif decition == '7': #Actualiza velocidad deseada
        motor.velocidad_deseada = (float(input("Velocidad deseada: "))/1)
        #motor.control_pid(motor.kp, motor.ki, motor.kd, setpoint = motor.velocidad_deseada)
    elif decition == '8':
        motor.motor.write(bytes("POS"+"\n", 'utf-8'))
        motor.leer()
    elif decition == '9':
        print(motor.posiciones)


print(motor.kp, motor.ki, motor.kd)
#from curses import baudrate
import serial
import time
import serial.tools.list_ports as port_list
from threading import Timer



'''ports = list(port_list.comports())
for p in ports:
    print (p)'''


class Sensor_profundidad:
    def __init__(self) -> None:
        self.conectar()
        pass

    def conectar(self):
        puerto = 'COM11'
        baud = 115200
        self.sensor = serial.Serial(port = puerto, baudrate = baud)

    def leer(self):
        leyendo = True
        palabra = b""
        while leyendo:
            ans = self.sensor.read()
            palabra += ans
            if ans == b'\n':
                leyendo = False
        return (self.decodificar(palabra))


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
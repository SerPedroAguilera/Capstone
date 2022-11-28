import serial
from time import time,sleep
import threading


class Velocidad(object):

	def __init__(self, port, kp, ki, kd):
		self.port = port
		self.baudrate = 115200
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.error_proporcional = 0
		self.error_integrador = 0
		self.error_derivativo = 0
		self.suma_error = 0
		self.suma_errores = 0
		self.tiempo_anterior = 0
		self.tiempo_actual = 0


	def pid(self, distancia_setpoint):
		self.distancia_setpoint = distancia_setpoint
		conexion = serial.Serial(self.port)
		conexion.baudrate = self.baudrate
		self.tiempo_anterior = time()
		sleep(1)
		self.tiempo_actual = time()
		
		if conexion.isOpen():
			print("Conectado")
			while True:
				dif_tiempo = self.tiempo_actual - self.tiempo_anterior
				linea = conexion.readline().decode('ascii')
				datos = linea.split(";")
				distancia = float(datos[1])
				error = distancia - self.distancia_setpoint 
				self.suma_error += error
				if self.suma_error > 6000:
					self.suma_error = 6000
				self.error_proporcional = self.kp * error
				self.error_integrador = self.ki * self.suma_error * dif_tiempo
				self.error_derivativo = self.kd * (error/dif_tiempo)
				self.suma_errores = self.error_derivativo + self.error_integrador + self.error_proporcional
				self.tiempo_anterior = self.tiempo_actual
				self.tiempo_actual = time()
				#print("distancia:" +str(distancia))
				#print("Suma error:" + str(self.suma_error))
				#print("Suma total:" + str(self.suma_errores))

	def get_error(self):
		return self.suma_errores
	
	def set_errores(self, valor):
		self.suma_errores = valor
		


if __name__ == '__main__':
	m1 = Velocidad('/dev/cu.usbserial-1420',2,0.01,0.6)
	x = threading.Thread(target = m1.pid , args=(100,))
	y = threading.Thread(target=m1.suma_errores, args=(1,))
	x.start()
	y.start()
	while True:
		print(y)







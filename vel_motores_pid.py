import serial
from time import time,sleep
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
		self.suma_errores = 0
		self.tiempo_anterior = 0
		self.tiempo_actual = 0


	def pid(self, velocidad_setpoint):
		self.velocidad = velocidad_setpoint
		conexion = serial.Serial(self.port)
		conexion.baudrate = self.baudrate
		self.tiempo_anterior = time()
		sleep(1)
		self.tiempo_actual = time()
		try:
			conexion.open()
		except Exception, e:
			print("Error al abrir el puerto:" + str(e))
			exit()
		if conexion.isOpen():
			while True:
				dif_tiempo = self.tiempo_actual - self.tiempo_anterior
				dato = conexion.readline()
				datos = dato.split(";")
				distancia = float(dato[1])
				error = self.velocidad - distancia
				self.error_proporcional = self.kp * error
				self.error_integrador = self.ki * error * dif_tiempo
				self.error_derivativo = self.kd * (error/dif_tiempo)
				
                self.tiempo_anterior = self.tiempo_actual
				self.tiempo_actual = time()





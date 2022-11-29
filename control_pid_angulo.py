import numpy as np
import time
from Center_mass import Centro_masa



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
            print('f')
            error = self.sensor.calcular_error()
            
            self.errores_angulo[2] = self.errores_angulo[1]
            self.errores_angulo[1] = self.errores_angulo[0]
            self.errores_angulo[0] = error
            salida = self.salida_anterior_angulo + (self.kp_angulo + self.dt*self.ki_angulo + self.kd_angulo/self.dt)*error + (-self.kp_angulo + -2*self.kd_angulo/self.dt)*self.errores_angulo[1] + (self.kd_angulo/self.dt)*self.errores_angulo[2]
            if abs(salida)<30:
                salida = 0
            
            self.salida_anterior_angulo = salida
            self.tiempo = time.time()
            print(salida)
            return salida
        return self.salida_anterior_angulo
            



'''vel_motor = 100
kp = 1
ki = 0
kd = 0
controlador_angulo = Control_angulo(kp, ki, kd)
a = input()

for i in range(1000):
    out = controlador_angulo.controlar_angulo()
    a = input()
        '''


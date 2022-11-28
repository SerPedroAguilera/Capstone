import numpy as np
import time
from Center_mass import Centro_masa



class Control():
    def __init__(self, kp, ki, kd) -> None:
        self.kp_angulo = kp
        self.ki_angulo = ki
        self.kd_angulo = kd
        self.errores_angulos = np.zeros(3)
        self.salida_anterior_angulo = 0
        self.tiempo = time.time()
        #self.periodo = 5
        self.dt = 0.5
        self.sensor = Centro_masa()
        pass

    def controlar_angulo(self):
        if (time.time()-self.tiempo) < self.dt:
            error = self.sensor.calcular_error()
            self.errores_angulo[2] = self.errores_angulo[1]
            self.errores_angulo[1] = self.errores_angulo[0]
            self.errores_angulo[0] = error
            salida = self.salida_anterior_angulo + (self.kp_angulo + self.dt*self.ki_angulo + self.kd_angulo/self.dt)*error + (-self.kp_angulo + -2*self.kd_angulo/self.dt)*self.errores_angulo[1] + (self.kd_angulo/self.dt)*self.errores_angulo[2]
            self.salida_anterior = salida
            return salida
            
controlador = Control(1, 1, 1)
controlador.controlar_angulo()

        


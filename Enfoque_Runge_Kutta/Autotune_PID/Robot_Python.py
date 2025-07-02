import numpy as np
from simple_pid import PID
import matplotlib.pyplot as plt

# Parámetros del robot
B = 0.2  # Ancho del robot
delta_t = 0.01  # Paso de tiempo
distancia_umbral = 0.1  # Distancia para considerar objetivo alcanzado

class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        self.vl = 0.0
        self.vr = 0.0
        
    def modelo(self, u1, u2):
        # RK4 para integración numérica
        def dx(vl, vr, phi):
            return (vl + vr)/2 * np.cos(phi)
        
        def dy(vl, vr, phi):
            return (vl + vr)/2 * np.sin(phi)
        
        def dphi(vl, vr):
            return (vr - vl)/B
        
        # Paso RK4
        k1_vl = u1
        k1_vr = u2
        k1_x = dx(self.vl, self.vr, self.phi)
        k1_y = dy(self.vl, self.vr, self.phi)
        k1_phi = dphi(self.vl, self.vr)
        
        k2_vl = u1
        k2_vr = u2
        k2_x = dx(self.vl + k1_vl*delta_t/2, self.vr + k1_vr*delta_t/2, self.phi + k1_phi*delta_t/2)
        k2_y = dy(self.vl + k1_vl*delta_t/2, self.vr + k1_vr*delta_t/2, self.phi + k1_phi*delta_t/2)
        k2_phi = dphi(self.vl + k1_vl*delta_t/2, self.vr + k1_vr*delta_t/2)
        
        k3_vl = u1
        k3_vr = u2
        k3_x = dx(self.vl + k2_vl*delta_t/2, self.vr + k2_vr*delta_t/2, self.phi + k2_phi*delta_t/2)
        k3_y = dy(self.vl + k2_vl*delta_t/2, self.vr + k2_vr*delta_t/2, self.phi + k2_phi*delta_t/2)
        k3_phi = dphi(self.vl + k2_vl*delta_t/2, self.vr + k2_vr*delta_t/2)
        
        k4_vl = u1
        k4_vr = u2
        k4_x = dx(self.vl + k3_vl*delta_t, self.vr + k3_vr*delta_t, self.phi + k3_phi*delta_t)
        k4_y = dy(self.vl + k3_vl*delta_t, self.vr + k3_vr*delta_t, self.phi + k3_phi*delta_t)
        k4_phi = dphi(self.vl + k3_vl*delta_t, self.vr + k3_vr*delta_t)
        
        self.x += (k1_x + 2*k2_x + 2*k3_x + k4_x) * delta_t / 6
        self.y += (k1_y + 2*k2_y + 2*k3_y + k4_y) * delta_t / 6
        self.phi += (k1_phi + 2*k2_phi + 2*k3_phi + k4_phi) * delta_t / 6
        self.vl += (k1_vl + 2*k2_vl + 2*k3_vl + k4_vl) * delta_t / 6
        self.vr += (k1_vr + 2*k2_vr + 2*k3_vr + k4_vr) * delta_t / 6
        
        # Normalizar ángulo
        self.phi = np.arctan2(np.sin(self.phi), np.cos(self.phi))

class AutoTunePID:
    def __init__(self, Kp=0.00003, Ki=0.000000001, Kd=0.0000000001):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.best_time = float('inf')
        self.step_size = 0.000001
        
    def tune(self, previous_error, integral, derivative, current_time):
        # Algoritmo de ajuste simple basado en gradiente descendente
        if current_time < self.best_time:
            self.best_time = current_time
            self.step_size *= 1.1
            self.Kp *= 1.1
            self.Ki *= 1.05
            self.Kd *= 1.01
        else:
            self.step_size *= 0.9
            self.Kp *= 0.9
            self.Ki *= 0.8
            self.Kd *= 0.7
            
        return PID(self.Kp, self.Ki, self.Kd, setpoint=0)

def simular_trayectoria(objetivo, max_time=60):
    robot = Robot()
    pid = PID(0.00003, 0.000000001, 0.0000000001, setpoint=0)
    tuner = AutoTunePID()
    
    tiempo = 0.0
    historial = []
    integral_error = 0.0
    prev_error = 0.0
    
    while tiempo < max_time:
        dx = objetivo[0] - robot.x
        dy = objetivo[1] - robot.y
        distancia = np.hypot(dx, dy)
        
        if distancia < distancia_umbral:
            break
            
        # Calcular error de orientación
        desired_angle = np.arctan2(dy, dx)
        error = desired_angle - robot.phi
        error = np.arctan2(np.sin(error), np.cos(error))  # Normalizar
        
        # Actualizar PID
        pid = tuner.tune(error, integral_error, (error - prev_error)/delta_t, tiempo)
        adjustment = pid(error)
        
        # Aplicar control
        u1 = 0.3 - adjustment
        u2 = 0.3 + adjustment
        
        # Limitar entradas
        u1 = np.clip(u1, -0.5, 0.5)
        u2 = np.clip(u2, -0.5, 0.5)
        
        # Actualizar modelo
        robot.modelo(u1, u2)
        
        # Registrar datos
        historial.append((robot.x, robot.y))
        tiempo += delta_t
        integral_error += error * delta_t
        prev_error = error
        
    return tiempo, np.array(historial)

# Ejecutar simulación
objetivo = (2.0, 3.0)
tiempo, trayectoria = simular_trayectoria(objetivo)

print(f"Tiempo para alcanzar el objetivo: {tiempo:.2f} segundos")

# Graficar trayectoria
plt.figure(figsize=(10, 6))
plt.plot(trayectoria[:,0], trayectoria[:,1], label='Trayectoria')
plt.scatter([0, objetivo[0]], [0, objetivo[1]], c='red', label='Puntos')
plt.xlabel('X')
plt.ylabel('Y')
plt.title(f'Trayectoria del robot - Tiempo: {tiempo:.2f}s')
plt.legend()
plt.grid(True)
plt.show()

# Guardar trayectoria en CSV
np.savetxt("trayectoria_python.csv", 
           np.column_stack((np.arange(0, tiempo, delta_t), trayectoria)),
           delimiter=",", 
           header="time,x,y", 
           comments="",
           fmt="%.4f")
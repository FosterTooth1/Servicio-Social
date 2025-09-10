import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution
import csv
from datetime import datetime
from multiprocessing import Pool, cpu_count
import os
import numpy as np
from scipy.optimize import differential_evolution
from functools import partial

# Configuración de parámetros

# Parámetros físicos del robot
wheel_diameter = 0.05  # 5 cm en metros
max_rpm = 300

# Cálculo de velocidad lineal máxima
radius = wheel_diameter / 2
max_rads = max_rpm * (2 * np.pi) / 60  # Convertir RPM a rad/s
max_wheel_velocity = radius * max_rads  # ≈ 0.7854 m/s

# Se leerán los waypoints desde un archivo CSV.
# El archivo debe tener el formato:
# x,y
# 3846.912496816170005e+03,-1.296688540366591951e+03
# 3847.579436709217589e+03,-1.296791357276932558e+03
# 3848.246314390027237e+03,-1.296894160712108487e+03

def load_waypoints(csv_filename):
    # Lee todas las columnas del CSV, omitiendo la cabecera
    return np.genfromtxt(csv_filename, delimiter=',', skip_header=1)

# Cargar waypoints
waypoints = load_waypoints("pista_escalada.csv")

# Actualizar la posición inicial del robot usando la primera coordenada del CSV.
# En el vector de estado, la posición se representa como [y, x] en las posiciones 3 y 4.
# Actualizar la posición inicial usando la ruta central (primeras dos columnas)
x0 = waypoints[0, 0]
y0 = waypoints[0, 1]

config = {
    'B': 0.2,              # Ancho del auto
    'dt': 0.01,            # Paso de tiempo (segundos)
    'N_steps': 1000000,      # Número máximo de pasos de simulación
    'Xe0': np.array([0, 0, 0, y0, x0]),  # Estado inicial ajustado al primer waypoint
    'base_acc': 0.1,       # Aceleración base (para avanzar)
    'max_acc': 0.7854      # Aceleración máxima permitida
}

dist_threshold = 0.5 # Umbral para considerar que se alcanzó un waypoint

# Controlador PID
class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.1, dt=0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0 # Término integral
        self.prev_error = 0.0 # Error anterior para derivada
        self.integral_limit = 1.0 # Límite para el término integral

    def reset(self):
        """Reinicia los términos integral y error anterior"""
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


# Dinámica del robot
def odefun(t, Xe, Xc, B):
    v_r, v_l, theta, y, x = Xe
    dx = (v_l + v_r) / 2 * np.cos(theta)
    dy = (v_l + v_r) / 2 * np.sin(theta)
    dtheta = (v_l - v_r) / B
    dv_r = Xc[1]
    dv_l = Xc[0]
    return np.array([dv_r, dv_l, dtheta, dy, dx])

def runge_kutta_step(t, Xe, Xc, dt, B):
    k1 = odefun(t, Xe, Xc, B)
    k2 = odefun(t + dt/2, Xe + k1 * dt/2, Xc, B)
    k3 = odefun(t + dt/2, Xe + k2 * dt/2, Xc, B)
    k4 = odefun(t + dt, Xe + k3 * dt, Xc, B)
    return Xe + (k1 + 2*k2 + 2*k3 + k4) * dt / 6

# Simulación principal
def lego_robot_simulation(params, waypoints, pid_params=None, visualize=True):
    B = params['B']
    dt = params['dt']
    N_steps = params['N_steps']
    Xe0 = params['Xe0']
    base_acc = params['base_acc']
    max_acc = params['max_acc']

    t_vec = np.linspace(0, dt * N_steps, N_steps)
    Xe = np.zeros((N_steps + 1, 5))
    Xe[0, :] = Xe0

    if pid_params is not None:
        pid = PIDController(*tuple(pid_params), dt=dt)
    else:
        pid = PIDController(dt=dt)

    Xc = np.zeros(2)
    total_error = 0.0
    current_wp_idx = 0

    if visualize:
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_title('Simulación del Robot con PID Autosintonizado')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

    for i in range(N_steps):
        theta = Xe[i, 2]
        pos_x = Xe[i, 4]
        pos_y = Xe[i, 3]

        if current_wp_idx >= len(waypoints):
            break

        wp = waypoints[current_wp_idx]
        error_distance = np.sqrt((wp[0]-pos_x)**2 + (wp[1]-pos_y)**2)

        # Actualizar waypoint si se alcanza
        if error_distance < dist_threshold:
            current_wp_idx += 1
            if current_wp_idx < len(waypoints):
                pid.reset()  # Reiniciar PID para el nuevo waypoint
            else:
                break
            # Procesa el nuevo waypoint inmediatamente

        if current_wp_idx >= len(waypoints):
            break

        # Calcular ángulo deseado y error con el nuevo waypoint
        wp = waypoints[current_wp_idx]
        desired_angle = np.arctan2(wp[1]-pos_y, wp[0]-pos_x)
        error_angle = (desired_angle - theta + np.pi) % (2*np.pi) - np.pi
        total_error += error_angle**2

        # Control adaptativo de velocidad
        adaptive_speed = base_acc * (1.0 - np.exp(-error_distance))
        control_correction = pid.update(error_angle)

        # Calcular aceleraciones con límites
        acc_left = adaptive_speed + control_correction
        acc_right = adaptive_speed - control_correction
        
        # Aplicar límites físicos
        acc_left = np.clip(acc_left, -max_acc, max_acc)
        acc_right = np.clip(acc_right, -max_acc, max_acc)
        Xc = np.array([acc_left, acc_right])

        # Integración del estado
        Xe[i+1, :] = runge_kutta_step(i*dt, Xe[i, :], Xc, dt, B)

        # Visualización
        if visualize and i % 10 == 0:
            update_plot(ax, Xe[i+1, :], wp, {}, dt)
            plt.pause(0.001)

    if visualize:
        plt.ioff()
        plt.show()
        
    return Xe[:i+1], t_vec[:i], total_error

# Función para actualizar gráfico
def update_plot(ax, Xe_current, waypoint, k_values, dt):
    pos_x = Xe_current[4]
    pos_y = Xe_current[3]
    theta = Xe_current[2]
    
    ax.clear()
    # Combinar todas las coordenadas X e Y de la ruta central y sus límites para definir los ejes
    x_all = np.concatenate([waypoints[:,0], waypoints[:,2], waypoints[:,4]])
    y_all = np.concatenate([waypoints[:,1], waypoints[:,3], waypoints[:,5]])
    ax.set_xlim(np.min(x_all) - 100, np.max(x_all) + 100)
    ax.set_ylim(np.min(y_all) - 100, np.max(y_all) + 100)
    ax.grid(True)
    
    # Dibujar la ruta central
    ax.plot(waypoints[:,0], waypoints[:,1], 'kx--', label='Ruta central')
    # Dibujar límite interno (segunda columna de par de coordenadas)
    ax.plot(waypoints[:,2], waypoints[:,3], 'b--', label='Límite interno')
    # Dibujar límite externo (tercera columna de par de coordenadas)
    ax.plot(waypoints[:,4], waypoints[:,5], 'g--', label='Límite externo')
    
    # Dibujar el waypoint actual (usando la ruta central)
    ax.plot(waypoint[0], waypoint[1], 'ro', markersize=10, label='Objetivo actual')
    # Dibujar la posición actual del robot
    ax.plot(pos_x, pos_y, 'bo', markersize=8, label='Posición actual')
    
    # Dibujar flecha de orientación
    arrow_length = 0.5
    ax.arrow(pos_x, pos_y,
             arrow_length*np.cos(theta),
             arrow_length*np.sin(theta),
             head_width=0.2, head_length=0.3,
             fc='g', ec='g', label='Orientación')
    
    ax.legend(loc='upper right')
    plt.draw()

# =============================
# Función principal
# =============================
def main():
    print("Cargando waypoints desde CSV...")
    # Los waypoints ya fueron cargados globalmente y la posición inicial actualizada en config.
    print(f"Waypoints cargados: {len(waypoints)} puntos")
    print("Iniciando autotuning PID...")
    #optimized_pid = autotune_pid(config, waypoints)
    optimized_pid = 7.1, 3.2, 3.36  # Valores de PID optimizados para la simulación
    print(f"\nParámetros óptimos encontrados:")
    print(f"Kp = {optimized_pid[0]:.2f}")
    print(f"Ki = {optimized_pid[1]:.2f}")
    print(f"Kd = {optimized_pid[2]:.2f}")
    
    # Guardar en CSV
    filename = "pid_optimized.csv"
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Escribir encabezado
        writer.writerow(["Timestamp", "Kp", "Ki", "Kd", "Waypoints"])
        
        # Escribir datos
        writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            f"{optimized_pid[0]:.2f}",
            f"{optimized_pid[1]:.2f}",
            f"{optimized_pid[2]:.2f}",
            len(waypoints)
        ])
    
    print(f"\nResultados guardados en {filename}")

    print("\nEjecutando simulación final con parámetros optimizados...")
    lego_robot_simulation(
        config, 
        waypoints, 
        pid_params=optimized_pid, 
        visualize=True
    )

if __name__ == "__main__":
    main()

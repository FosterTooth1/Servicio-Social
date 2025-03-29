import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution
import csv
from datetime import datetime

# =============================
# Configuración de parámetros
# =============================

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
    # Usamos numpy para leer el CSV, omitiendo la cabecera
    return np.genfromtxt(csv_filename, delimiter=',', skip_header=1)

# Cargar waypoints
waypoints = load_waypoints("pista_escalada.csv")

# Actualizar la posición inicial del robot usando la primera coordenada del CSV.
# En el vector de estado, la posición se representa como [y, x] en las posiciones 3 y 4.
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

dist_threshold = 0.5

# =============================
# Controlador PID mejorado
# =============================
class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.1, dt=0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = 1.0  # Límite anti-windup

    def reset(self):
        """Reinicia los términos integral y error anterior"""
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        # Anti-windup
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


# =============================
# Función de autotuning con evolucion diferencial
# =============================

def autotune_pid(config, waypoints):
    def objective(pid_params):
        Kp, Ki, Kd = pid_params
        print(f"Probando parámetros: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
        try:
            Xe, t, _ = lego_robot_simulation(
                config, waypoints, 
                pid_params=(Kp, Ki, Kd), 
                visualize=False
            )
            
            # Calcular métricas de desempeño
            pos_errors = []
            angle_errors = []
            current_wp_idx = 0
            
            for i in range(len(t)):
                if current_wp_idx >= len(waypoints):
                    break
                
                pos_x = Xe[i, 4]
                pos_y = Xe[i, 3]
                theta = Xe[i, 2]
                wp = waypoints[current_wp_idx]
                
                error_distance = np.sqrt((wp[0]-pos_x)**2 + (wp[1]-pos_y)**2)
                pos_errors.append(error_distance)
                
                if error_distance < dist_threshold:
                    current_wp_idx += 1
                    continue
                
                desired_angle = np.arctan2(wp[1]-pos_y, wp[0]-pos_x)
                error_angle = (desired_angle - theta + np.pi) % (2*np.pi) - np.pi
                angle_errors.append(error_angle**2)
            
            if not pos_errors:
                return 1e6  # Penalizar si no se movió
            
            # Combinar métricas con pesos
            metric = np.mean(pos_errors) + 0.5 * np.mean(angle_errors)
            print(f"Error total: {metric:.2f}")
            
            return metric
            
        except Exception as e:
            print(f"Error en simulación: {str(e)}")
            return 1e6

    # Límites realistas para los parámetros
    bounds = [
        (0.1, 10.0),   # Kp
        (0.0, 5.0),    # Ki
        (0.0, 5.0)     # Kd
    ]
    
    # Optimización con algoritmo evolutivo
    result = differential_evolution(
        objective,
        bounds,
        strategy='best1bin',
        maxiter=50,
        popsize=30,
        mutation=(0.5, 1.0),
        recombination=0.9,
        seed=42,
        tol=0.01,
        polish=True
    )
    
    if not result.success:
        print("No se encontró convergencia durante la optimización. Se utilizarán los parámetros con menor costo encontrados.")
        # Se intenta la simulación final con los mejores parámetros obtenidos
        best_params = result.x
        try:
            Xe, t, _ = lego_robot_simulation(
                config, waypoints, 
                pid_params=tuple(best_params), 
                visualize=False
            )
            print("Simulación completada con los mejores parámetros encontrados.")
        except Exception as e:
            print(f"Error al ejecutar la simulación final: {str(e)}")
        return best_params
    else:
        return result.x
"""

import numpy as np

def autotune_pid(config, waypoints):
    def objective(pid_params):
        Kp, Ki, Kd = pid_params
        print(f"Probando parámetros: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
        try:
            Xe, t, _ = lego_robot_simulation(
                config, waypoints, 
                pid_params=(Kp, Ki, Kd), 
                visualize=False
            )
            
            # Calcular métricas de desempeño
            pos_errors = []
            angle_errors = []
            current_wp_idx = 0
            dist_threshold = 0.1  # Asegurar que esté definido
            
            for i in range(len(t)):
                if current_wp_idx >= len(waypoints):
                    break
                
                pos_x = Xe[i, 4]
                pos_y = Xe[i, 3]
                theta = Xe[i, 2]
                wp = waypoints[current_wp_idx]
                
                error_distance = np.sqrt((wp[0]-pos_x)**2 + (wp[1]-pos_y)**2)
                pos_errors.append(error_distance)
                
                if error_distance < dist_threshold:
                    current_wp_idx += 1
                    continue
                
                desired_angle = np.arctan2(wp[1]-pos_y, wp[0]-pos_x)
                error_angle = (desired_angle - theta + np.pi) % (2*np.pi) - np.pi
                angle_errors.append(error_angle**2)
            
            if not pos_errors:
                return 1e6  # Penalizar si no se movió
            
            # Combinar métricas con pesos
            metric = np.mean(pos_errors) + 0.5 * np.mean(angle_errors)
            print(f"Error total: {metric:.2f}")
            
            return metric
            
        except Exception as e:
            print(f"Error en simulación: {str(e)}")
            return 1e6

    # Límites para los parámetros PID
    bounds = [
        (0.1, 10.0),   # Kp
        (0.0, 5.0),    # Ki
        (0.0, 5.0)     # Kd
    ]
    
    # Parámetros del algoritmo genético
    num_generaciones = 1000  # Número de generaciones
    num_pob = 500            # Tamaño de la población
    Pc = 0.9              # Probabilidad de cruzamiento
    Pm = 0.15              # Probabilidad de mutación por variable
    num_var = 3           # Número de variables (Kp, Ki, Kd)
    
    limite_inferior = np.array([b[0] for b in bounds])
    limite_superior = np.array([b[1] for b in bounds])
    
    # Inicializar población
    poblacion = np.random.uniform(
        low=limite_inferior,
        high=limite_superior,
        size=(num_pob, num_var)
    )
    
    best_global = None
    best_fitness = np.inf
    
    for generacion in range(num_generaciones):
        # Ajustar parámetros evolutivos según generación
        if generacion < num_generaciones * 0.5:
            Nc, Nm = 2, 20
        elif generacion < num_generaciones * 0.75:
            Nc, Nm = 5, 50
        elif generacion < num_generaciones * 0.8:
            Nc, Nm = 10, 75
        elif generacion < num_generaciones * 0.95:
            Nc, Nm = 15, 85
        else:
            Nc, Nm = 20, 100
        
        # Evaluar aptitud
        aptitud = np.zeros(num_pob)
        for i in range(num_pob):
            aptitud[i] = objective(poblacion[i])
        
        # Actualizar mejor global
        current_best_idx = np.argmin(aptitud)
        current_best_fitness = aptitud[current_best_idx]
        current_best_ind = poblacion[current_best_idx].copy()
        
        if current_best_fitness < best_fitness:
            best_fitness = current_best_fitness
            best_global = current_best_ind.copy()
        
        # Selección por torneo binario
        padres = np.zeros_like(poblacion)
        torneo = np.array([np.random.permutation(num_pob), 
                          np.random.permutation(num_pob)]).T
        
        for i in range(num_pob):
            a, b = torneo[i]
            if aptitud[a] < aptitud[b]:
                padres[i] = poblacion[a]
            else:
                padres[i] = poblacion[b]
        
        # Cruzamiento SBX
        hijos = np.zeros_like(padres)
        for i in range(0, num_pob, 2):
            if np.random.rand() <= Pc:
                hijo1, hijo2 = [], []
                for j in range(num_var):
                    p1, p2 = padres[i, j], padres[i+1, j]
                    # Asegurar p1 <= p2 para evitar divisiones negativas
                    if p1 > p2:
                        p1, p2 = p2, p1
                    # Calcular beta
                    beta = 1 + (2/(p2 - p1)) * min(p1 - limite_inferior[j], 
                                                    limite_superior[j] - p2)
                    alpha = 2 - abs(beta) ** -(Nc + 1)
                    U = np.random.rand()
                    
                    if U <= 1/alpha:
                        beta_c = (U * alpha) ** (1/(Nc + 1))
                    else:
                        beta_c = (1/(2 - U * alpha)) ** (1/(Nc + 1))
                    
                    h1 = 0.5 * ((p1 + p2) - beta_c * abs(p2 - p1))
                    h2 = 0.5 * ((p1 + p2) + beta_c * abs(p2 - p1))
                    # Asegurar límites
                    h1 = np.clip(h1, limite_inferior[j], limite_superior[j])
                    h2 = np.clip(h2, limite_inferior[j], limite_superior[j])
                    hijo1.append(h1)
                    hijo2.append(h2)
                hijos[i] = hijo1
                hijos[i+1] = hijo2
            else:
                hijos[i] = padres[i]
                hijos[i+1] = padres[i+1]
        
        # Mutación polinomial
        for i in range(num_pob):
            for j in range(num_var):
                if np.random.rand() <= Pm:
                    val = hijos[i, j]
                    delta = min(val - limite_inferior[j], 
                                limite_superior[j] - val) / (limite_superior[j] - limite_inferior[j])
                    r = np.random.rand()
                    
                    if r <= 0.5:
                        delta_q = (2*r + (1 - 2*r)*(1 - delta)**(Nm + 1))**(1/(Nm + 1)) - 1
                    else:
                        delta_q = 1 - (2*(1 - r) + 2*(r - 0.5)*(1 - delta)**(Nm + 1))**(1/(Nm + 1))
                    
                    val += delta_q * (limite_superior[j] - limite_inferior[j])
                    val = np.clip(val, limite_inferior[j], limite_superior[j])
                    hijos[i, j] = val
        
        # Elitismo: Reemplazar un individuo aleatorio con el mejor actual
        idx = np.random.randint(num_pob)
        hijos[idx] = current_best_ind
        poblacion = hijos.copy()
    
    # Simulación final con mejores parámetros
    best_params = best_global
    print("Optimización completada. Probando mejores parámetros...")
    try:
        Xe, t, _ = lego_robot_simulation(
            config, waypoints, 
            pid_params=tuple(best_params), 
            visualize=False
        )
        print("Simulación exitosa con mejores parámetros.")
    except Exception as e:
        print(f"Error en simulación final: {str(e)}")
    
    return best_params
"""

# =============================
# Dinámica del robot (sin cambios)
# =============================
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

# =============================
# Simulación principal mejorada
# =============================
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
        acc_left = np.clip(acc_left, 0.0, max_acc)
        acc_right = np.clip(acc_right, 0.0, max_acc)
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

# =============================
# Función para actualizar gráfico
# =============================
def update_plot(ax, Xe_current, waypoint, k_values, dt):
    pos_x = Xe_current[4]
    pos_y = Xe_current[3]
    theta = Xe_current[2]

    ax.clear()
    # Ajustar límites para visualizar los waypoints grandes, o bien escalarlos según se requiera.
    ax.set_xlim(np.min(waypoints[:,0]) - 100, np.max(waypoints[:,0]) + 100)
    ax.set_ylim(np.min(waypoints[:,1]) - 100, np.max(waypoints[:,1]) + 100)
    ax.grid(True)
    
    # Dibujar trayectoria planeada
    ax.plot(waypoints[:,0], waypoints[:,1], 'kx--', label='Ruta planeada')
    ax.plot(waypoint[0], waypoint[1], 'ro', markersize=10, label='Objetivo actual')
    
    # Dibujar robot
    ax.plot(pos_x, pos_y, 'bo', markersize=8, label='Posición actual')
    
    # Dibujar orientación
    arrow_length = 0.5
    ax.arrow(pos_x, pos_y, 
             arrow_length*np.cos(theta), arrow_length*np.sin(theta),
             head_width=0.2, head_length=0.3, fc='g', ec='g', label='Orientación')
    
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
    optimized_pid = autotune_pid(config, waypoints)
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

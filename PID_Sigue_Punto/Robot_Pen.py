import numpy as np
import matplotlib.pyplot as plt

# =============================
# Configuración de parámetros
# =============================
config = {
    'B': 0.2,         # Ancho del auto
    'dt': 0.01,       # Paso de tiempo (segundos)
    'N_steps': 1000,  # Número de pasos de simulación
    # Condición inicial: [velocidad rueda derecha, velocidad rueda izquierda, ángulo de giro, posición y, posición x]
    'Xe0': np.array([0, 0, 0, 0, 0]),
    # Control: aceleración de las ruedas para cada paso
    'acc_right': 0.5,  # Aceleración rueda derecha
    'acc_left': 0.5     # Aceleración rueda izquierda
}

# =============================
# Función que define la dinámica del robot
# =============================
def odefun(t, Xe, Xc, B):
    """
    Calcula la derivada del vector de estado.

    Parámetros:
      t  : tiempo (no se utiliza en este caso)
      Xe : vector de estado [v_r, v_l, theta, y, x]
      Xc : vector de control [aceleración rueda izquierda, aceleración rueda derecha]
      B  : ancho del auto

    Retorna:
      dXedt : derivada del vector de estado
    """
    # Desempaquetar estado
    v_r, v_l, theta, y, x = Xe

    # Cálculos de derivadas
    dx = (v_l + v_r) / 2 * np.cos(theta)   # Velocidad en x
    dy = (v_l + v_r) / 2 * np.sin(theta)   # Velocidad en y
    dtheta = (v_l - v_r) / B               # Gradiente del ángulo de giro
    dv_r = Xc[1]                         # Aceleración rueda derecha
    dv_l = Xc[0]                         # Aceleración rueda izquierda

    return np.array([dv_r, dv_l, dtheta, dy, dx])

# =============================
# Función para un paso de integración Runge-Kutta 4
# =============================
def runge_kutta_step(t, Xe, Xc, dt, B):
    """
    Realiza un paso de integración usando Runge-Kutta de 4to orden.

    Parámetros:
      t  : tiempo actual
      Xe : vector de estado actual
      Xc : vector de control actual
      dt : paso de tiempo
      B  : ancho del auto

    Retorna:
      Xe_next : vector de estado en el siguiente paso
    """
    k1 = odefun(t, Xe, Xc, B)
    k2 = odefun(t + dt/2, Xe + k1 * dt/2, Xc, B)
    k3 = odefun(t + dt/2, Xe + k2 * dt/2, Xc, B)
    k4 = odefun(t + dt, Xe + k3 * dt, Xc, B)

    return Xe + (k1 + 2*k2 + 2*k3 + k4) * dt / 6

# =============================
# Función para actualizar la gráfica
# =============================
def update_plot(ax, Xe_current, k_values, dt):
    """
    Actualiza la visualización del robot.

    Parámetros:
      ax         : objeto Axes de Matplotlib
      Xe_current : vector de estado actual [v_r, v_l, theta, y, x]
      k_values   : diccionario con los k-values de Runge-Kutta (opcional para derivar la velocidad)
      dt         : paso de tiempo (utilizado para calcular la velocidad promedio)
    """
    # Extraer posición (recordando que la posición x es el quinto elemento y la y es el cuarto)
    pos_x = Xe_current[4]
    pos_y = Xe_current[3]

    # Se puede calcular la velocidad promedio en la iteración (si se pasan k_values)
    # En este ejemplo se calcula a partir de k1, k2, k3 y k4 para los componentes de posición.
    # Si no se dispone de k_values, se omite la flecha.
    if k_values:
        vx = (k_values['k1'][4] + 2*k_values['k2'][4] + 2*k_values['k3'][4] + k_values['k4'][4]) / 6
        vy = (k_values['k1'][3] + 2*k_values['k2'][3] + 2*k_values['k3'][3] + k_values['k4'][3]) / 6
    else:
        vx, vy = 0, 0

    ax.clear()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.grid(True)
    # Dibujar posición como punto azul
    ax.plot(pos_x, pos_y, 'bo')
    # Dibujar la flecha que indica la dirección y magnitud de la velocidad
    ax.quiver(pos_x, pos_y, vx, vy, angles='xy', scale_units='xy', scale=0.5, color='r', linewidth=2)
    plt.pause(0.0001)

# =============================
# Función principal de simulación
# =============================
def lego_robot_simulation(params):
    # Extraer parámetros
    B = params['B']
    dt = params['dt']
    N_steps = params['N_steps']
    Xe0 = params['Xe0']
    # Crear los vectores de control para cada rueda
    # En la simulación se asume que el control es constante en cada paso
    ur = np.ones(N_steps) * params['acc_right']  # aceleración rueda derecha
    ul = np.ones(N_steps) * params['acc_left']   # aceleración rueda izquierda
    # Cada fila de Xc contiene [aceleración rueda izquierda, aceleración rueda derecha]
    Xc = np.column_stack((ul, ur))

    # Vector de tiempos
    t_vec = np.linspace(0, dt * N_steps, N_steps)

    # Inicialización del vector de estado
    Xe = np.zeros((N_steps + 1, 5))
    Xe[0, :] = Xe0

    # Configuración de la figura para la visualización
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.grid(True)

    # Bucle de simulación
    for i in range(N_steps):
        t_current = t_vec[i]
        # Se guardan los k-values para actualizar la gráfica
        k1 = odefun(t_current, Xe[i, :], Xc[i, :], B)
        k2 = odefun(t_current + dt/2, Xe[i, :] + k1 * dt/2, Xc[i, :], B)
        k3 = odefun(t_current + dt/2, Xe[i, :] + k2 * dt/2, Xc[i, :], B)
        k4 = odefun(t_current + dt, Xe[i, :] + k3 * dt, Xc[i, :], B)

        # Actualizar el estado con RK4
        Xe[i+1, :] = Xe[i, :] + (k1 + 2*k2 + 2*k3 + k4) * dt / 6

        # Actualización de la gráfica
        k_values = {'k1': k1, 'k2': k2, 'k3': k3, 'k4': k4}
        update_plot(ax, Xe[i+1, :], k_values, dt)

    plt.ioff()
    plt.show()
    return Xe, t_vec

# =============================
# Función main
# =============================
def main():
    """
    Función principal para ejecutar la simulación.
    Se pueden modificar los parámetros en el diccionario 'config' para ajustar la simulación.
    """
    Xe, t_vec = lego_robot_simulation(config)

if __name__ == "__main__":
    main()

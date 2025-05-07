import numpy as np
from random import uniform

# FUNCIONES DEL ALGORITMO GENÉTICO
def inicializar_poblacion(mi_path, num_pob, num_var, largo_real, ancho_real):
    """
    Inicializa la población de trayectorias para un algoritmo genético.
    Cada individuo representa un punto en la pista, con desplazamiento lateral aleatorio.
    """
    puntos = []
    largo_svg = mi_path.length()
    escala_factor = largo_real / largo_svg
    print(f"Largo SVG: {largo_svg:.2f} | Largo real: {largo_real} m | Escala: {escala_factor:.4f}")
    print(f"La longitud de la pista es de: {largo_svg}")

    # Distribución del total de puntos con el largo de la pista
    distancias = np.linspace(0, largo_svg, num=num_pob)

    # Obtener los puntos usando ilength (inversa de la longitud acumulada -> t)
    t_values = [mi_path.ilength(d) for d in distancias]

    for t in t_values:
        punto = mi_path.point(t)
        direccion = mi_path.derivative(t)

        # Normalizar el vector de dirección
        dx = direccion.real
        dy = direccion.imag
        mag = np.hypot(dx, dy)
        if mag == 0:
            continue  # Evitar divisiones entre 0 

        # Vector normal (perpendicular) al camino en ese punto
        normal_x = -dy / mag
        normal_y = dx / mag

        # Desplazamiento lateral aleatorio (ai ∈ [0, ancho_pista])
        ai = uniform(10, ancho_real-10)

        # Nuevo punto desplazado lateralmente 
        punto_x_escalado = punto.real * escala_factor
        punto_y_escalado = punto.imag * escala_factor
        x_lateral = (punto_x_escalado + normal_x * (-ai/2)) ## -> Se le cambió el signo + por el -, para que los puntos estén dentro de la pista.
        y_lateral = (-punto_y_escalado + normal_y * (ai/2)) # Se invierte Y por convención gráfica

        puntos.append((x_lateral, y_lateral))

    return puntos


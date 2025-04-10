import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths

# Parámetros
ancho_extra = 10  # Ancho de la línea extra (no se usa en este ejemplo)
num_points = 120  # Número deseado de puntos (puedes cambiarlo a 50, 500, 1000, etc.)

# Cargar el archivo SVG
svg_file = "Pistas Limpias/Autódromo_Hermanos_Rodríguez_2015 (Limpia).svg"
paths, attributes = svg2paths(svg_file)

# Extraer coordenadas del path
x_coords = []
y_coords = []

for path in paths:
    for segment in path:
        # Discretizar la curva en puntos
        points = np.linspace(0, 1, num=10000)
        for t in points:
            point = segment.point(t)
            x_coords.append(point.real)  # Parte real como X
            y_coords.append(-point.imag)  # Parte imaginaria como Y (invertimos el eje Y)

# Convertir a numpy arrays
x_coords = np.array(x_coords)
y_coords = np.array(y_coords)

# Calcular la longitud total del circuito en su escala original
def calculate_length(x, y):
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)  # Distancias entre puntos consecutivos
    return np.sum(distances)  # Sumar todas las distancias

original_length = calculate_length(x_coords, y_coords)
print(f"Longitud original del circuito: {original_length:.2f} unidades")

# Escalar el circuito a la longitud deseada
desired_length = 25647  # Longitud deseada
scale_factor = desired_length / original_length
print(f"Factor de escala: {scale_factor:.6f}")

# Aplicar el factor de escala
x_scaled = x_coords * scale_factor
y_scaled = y_coords * scale_factor

# Reescalar de una escala de 10 metros a 1 metro
x_final = x_scaled / 10
y_final = y_scaled / 10

# Submuestrear la cantidad deseada de puntos
indices = np.linspace(0, len(x_final) - 1, num=num_points, dtype=int)
x_sampled = x_final[indices]
y_sampled = y_final[indices]

# Graficar el circuito escalado y reescalado
plt.figure(figsize=(10, 10))
plt.plot(x_final, y_final, color="lightgray", linewidth=1, label="Todo el circuito")
plt.plot(x_sampled, y_sampled, color="r", linewidth=2, label=f"{num_points} puntos")
plt.axis("equal")
plt.legend()
plt.show()

# Guardar las coordenadas submuestreadas en un archivo CSV
np.savetxt("pista_escalada.csv", np.column_stack((x_sampled, y_sampled)), 
           delimiter=",", header="x,y", comments="")

import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths

# Parámetros
ancho_extra = 1  # Ancho de la línea extra

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

# Escalar el circuito
desired_length = 25647  # Longitud deseada
scale_factor = desired_length / original_length
print(f"Factor de escala: {scale_factor:.6f}")

# Aplicar el factor de escala
x_scaled = x_coords * scale_factor / 10  # Reescalar de una escala de 10 metros a 1 metro
y_scaled = y_coords * scale_factor / 10  # Reescalar de una escala de 10 metros a 1 metro

# Calcular los vectores tangentes
dx = np.gradient(x_scaled)
dy = np.gradient(y_scaled)

# Normalizar los vectores tangentes para obtener la dirección perpendicular
magnitudes = np.sqrt(dx**2 + dy**2)
nx = -dy / magnitudes  # Componente x de la normal
ny = dx / magnitudes   # Componente y de la normal

# Crear la línea desplazada
x_offset = x_scaled + nx * (ancho_extra / 2)
y_offset = y_scaled + ny * (ancho_extra / 2)

# Graficar el circuito escalado con la segunda línea
plt.figure(figsize=(8, 6))
plt.plot(x_scaled, y_scaled, 'k-', linewidth=3, label="Línea interna")
plt.plot(x_offset, y_offset, 'r-', linewidth=3, label="Línea externa")
plt.axis("equal")  # Mantener proporción
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title(f"Circuito Escalado a {desired_length} m con Ancho Extra de {ancho_extra} m")
plt.legend()
plt.grid(True)
plt.show()
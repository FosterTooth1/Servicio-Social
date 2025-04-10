import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths

# Parámetros
ancho_extra = 1        # Ancho total de la pista (desplazamiento total entre ambos bordes será ancho_extra)
num_points = 120       # Número deseado de puntos para la línea central
desired_length = 25647 # Longitud deseada para la pista

# Cargar el archivo SVG
svg_file = "Pistas Limpias/Autódromo_Hermanos_Rodríguez_2015 (Limpia).svg"
paths, attributes = svg2paths(svg_file)

# Extraer coordenadas del path
x_coords = []
y_coords = []

for path in paths:
    for segment in path:
        # Discretizar la curva en 10.000 puntos
        points = np.linspace(0, 1, num=10000)
        for t in points:
            point = segment.point(t)
            x_coords.append(point.real)  # Parte real como X
            y_coords.append(-point.imag) # Parte imaginaria como Y (invertimos el eje Y)

# Convertir a arrays numpy
x_coords = np.array(x_coords)
y_coords = np.array(y_coords)

# Función para calcular la longitud total de la pista
def calculate_length(x, y):
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    return np.sum(distances)

original_length = calculate_length(x_coords, y_coords)
print(f"Longitud original del circuito: {original_length:.2f} unidades")

# Calcular el factor de escala y aplicarlo
scale_factor = desired_length / original_length
print(f"Factor de escala: {scale_factor:.6f}")

# Escalar y reescalar (de 10 m a 1 m)
x_full = (x_coords * scale_factor) / 10
y_full = (y_coords * scale_factor) / 10

# Calcular vectores tangentes y normales en cada punto de la pista completa
dx = np.gradient(x_full)
dy = np.gradient(y_full)
magnitudes = np.sqrt(dx**2 + dy**2)
# Evitar división por cero
magnitudes[magnitudes==0] = 1e-10
nx = -dy / magnitudes  # Normal (componente x)
ny = dx / magnitudes   # Normal (componente y)

# Submuestrear la cantidad deseada de puntos para la línea central
indices = np.linspace(0, len(x_full) - 1, num=num_points, dtype=int)
x_central = x_full[indices]
y_central = y_full[indices]
nx_sub = nx[indices]
ny_sub = ny[indices]

# Generar dos bordes continuos desplazando la línea central en ambas direcciones
# Se desplaza medio ancho_extra hacia cada lado
x_border1 = x_central + (ancho_extra / 2) * nx_sub
y_border1 = y_central + (ancho_extra / 2) * ny_sub
x_border2 = x_central - (ancho_extra / 2) * nx_sub
y_border2 = y_central - (ancho_extra / 2) * ny_sub

# Graficar la pista: línea central y dos bordes
plt.figure(figsize=(10, 10))
plt.plot(x_full, y_full, color="lightgray", linewidth=1, label="Pista completa")
plt.plot(x_central, y_central, 'r-', linewidth=2, label="Línea central")
plt.plot(x_border1, y_border1, 'b-', linewidth=2, label="Borde 1")
plt.plot(x_border2, y_border2, 'g-', linewidth=2, label="Borde 2")
plt.axis("equal")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Pista de carreras con línea central y dos bordes")
plt.legend()
plt.grid(True)
plt.show()

# Guardar en CSV: cada columna contiene los datos de la línea central y los bordes
# El archivo contendrá las columnas: x_central, y_central, x_border1, y_border1, x_border2, y_border2
all_data = np.column_stack((x_central, y_central, x_border1, y_border1, x_border2, y_border2))
np.savetxt("pista_completa.csv", all_data, delimiter=",", header="x_central,y_central,x_border1,y_border1,x_border2,y_border2", comments="")

print("Archivo CSV 'pista_completa.csv' guardado con éxito.")

import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths
from scipy.interpolate import interp1d  # Importar interp1d para interpolación
# Para el diálogo de selección de archivo
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# Parámetros
ancho_extra = 1        # Ancho total de la pista
num_points = 1000       # Número de puntos para la línea central
desired_length = 400    # Longitud deseada para la pista

# Cargar el archivo SVG
root = Tk()
root.withdraw()
# Abre el diálogo y permite seleccionar solo archivos .svg
svg_file = askopenfilename(
    title="Selecciona el archivo SVG",
    filetypes=[("Archivos SVG", "*.svg"), ("Todos los archivos", "*.*")]
)
root.destroy()
if not svg_file:
    raise FileNotFoundError("No se ha seleccionado ningún archivo SVG.")

# Leer el archivo SVG y extraer los paths
paths, attributes = svg2paths(svg_file)

# Extraer coordenadas del path
x_coords = []
y_coords = []

for path in paths:
    for segment in path:
        points = np.linspace(0, 1, num=10000)
        for t in points:
            point = segment.point(t)
            x_coords.append(point.real)
            y_coords.append(-point.imag)  # Invertir eje Y

x_coords = np.array(x_coords)
y_coords = np.array(y_coords)

# Función para calcular la longitud total
def calculate_length(x, y):
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    return np.sum(distances)

original_length = calculate_length(x_coords, y_coords)
scale_factor = desired_length / original_length

# Escalar todas las coordenadas
x_full = x_coords * scale_factor
y_full = y_coords * scale_factor

# Calcular la longitud acumulada para parametrización por longitud de arco
dx_full = np.diff(x_full)
dy_full = np.diff(y_full)
distances = np.sqrt(dx_full**2 + dy_full**2)
cumulative_distances = np.zeros(len(x_full))
cumulative_distances[1:] = np.cumsum(distances)
total_length = cumulative_distances[-1]

# Generar puntos equidistantes a lo largo de la longitud de arco
s_new = np.linspace(0, total_length, num_points)

# Interpolar coordenadas para obtener la línea central remuestreada
interp_x = interp1d(cumulative_distances, x_full, kind='linear')
interp_y = interp1d(cumulative_distances, y_full, kind='linear')
x_central = interp_x(s_new)
y_central = interp_y(s_new)

# Calcular vectores tangentes (dx, dy) para los puntos remuestreados
dx = np.gradient(x_central)
dy = np.gradient(y_central)
magnitudes = np.sqrt(dx**2 + dy**2)
magnitudes[magnitudes == 0] = 1e-10  # Evitar división por cero

# Calcular vectores normales
nx = -dy / magnitudes
ny = dx / magnitudes

# Generar los bordes desplazando la línea central
x_border1 = x_central + (ancho_extra / 2) * nx
y_border1 = y_central + (ancho_extra / 2) * ny
x_border2 = x_central - (ancho_extra / 2) * nx
y_border2 = y_central - (ancho_extra / 2) * ny

# Graficar la pista
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

# Guardar en CSV
all_data = np.column_stack((x_central, y_central, x_border1, y_border1, x_border2, y_border2))
np.savetxt("Pista_completa.csv", all_data, delimiter=",", header="x_central,y_central,x_border1,y_border1,x_border2,y_border2", comments="")

print("Archivo CSV 'Pista_completa.csv' guardado con éxito.")
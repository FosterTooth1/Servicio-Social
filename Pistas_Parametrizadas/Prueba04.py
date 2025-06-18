import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths
from scipy.interpolate import interp1d
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# Parámetros
ancho_extra   = 1        # Ancho total de la pista
num_points    = 600      # Número de puntos para la línea central
desired_length= 400      # Longitud deseada para la pista

# ——— Selección de archivo SVG vía diálogo ———
root = Tk()
root.withdraw()
svg_file = askopenfilename(
    title="Selecciona un archivo SVG",
    filetypes=[("SVG files", "*.svg"), ("All files", "*.*")]
)
if not svg_file:
    print("No se seleccionó ningún archivo. Saliendo.")
    exit()

# Cargar el SVG
paths, attributes = svg2paths(svg_file)

# Extraer coordenadas del path
x_coords, y_coords = [], []
for path in paths:
    for segment in path:
        for t in np.linspace(0, 1, num=10000):
            pt = segment.point(t)
            x_coords.append(pt.real)
            y_coords.append(-pt.imag)  # invertimos Y

x_coords = np.array(x_coords)
y_coords = np.array(y_coords)

# Función para calcular longitud
def calculate_length(x, y):
    d = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    return d.sum()

original_length = calculate_length(x_coords, y_coords)
scale_factor    = desired_length / original_length

# Escalado
x_full = x_coords * scale_factor
y_full = y_coords * scale_factor

# Longitud acumulada para parametrización por arco
distances = np.sqrt(np.diff(x_full)**2 + np.diff(y_full)**2)
cum_dist  = np.concatenate([[0], np.cumsum(distances)])
total_len = cum_dist[-1]

# Puntos equidistantes
s_new = np.linspace(0, total_len, num_points)
interp_x = interp1d(cum_dist, x_full, kind='linear')
interp_y = interp1d(cum_dist, y_full, kind='linear')
x_central = interp_x(s_new)
y_central = interp_y(s_new)

# Tangentes y normales
dx = np.gradient(x_central)
dy = np.gradient(y_central)
mag = np.hypot(dx, dy)
mag[mag==0] = 1e-10
nx = -dy / mag
ny = dx / mag

# Bordes
x_border1 = x_central + (ancho_extra/2)*nx
y_border1 = y_central + (ancho_extra/2)*ny
x_border2 = x_central - (ancho_extra/2)*nx
y_border2 = y_central - (ancho_extra/2)*ny

# Graficar
plt.figure(figsize=(10,10))
plt.plot(x_full, y_full,   color="lightgray", linewidth=1, label="Pista completa")
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

# Guardar CSV
all_data = np.column_stack((x_central, y_central, x_border1, y_border1, x_border2, y_border2))
out_csv = "pista_completa.csv"
np.savetxt(
    out_csv, all_data, delimiter=",",
    header="x_central,y_central,x_border1,y_border1,x_border2,y_border2",
    comments=""
)
print(f"Archivo CSV '{out_csv}' guardado con éxito.")

import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths, Path, Line, CubicBezier, QuadraticBezier, Arc
from scipy.interpolate import interp1d
from tkinter import Tk, filedialog
import csv
from math import isclose

# Función para determinar si un segmento es recto
def is_straight_segment(segment, tolerance=1e-3):
    """Determina si un segmento es una recta basado en su curvatura"""
    # Para segmentos de línea, siempre son rectos
    if isinstance(segment, Line):
        return True
    
    # Para segmentos Bezier cúbicos
    if isinstance(segment, CubicBezier):
        # Calcular vectores de dirección en los puntos inicial y final
        start_dir = (segment.control1.real - segment.start.real, 
                     segment.control1.imag - segment.start.imag)
        end_dir = (segment.end.real - segment.control2.real, 
                   segment.end.imag - segment.control2.imag)
        
        # Calcular la dirección general del segmento
        total_dir = (segment.end.real - segment.start.real, 
                     segment.end.imag - segment.start.imag)
        
        # Normalizar vectores
        length_total = np.sqrt(total_dir[0]**2 + total_dir[1]**2)
        if length_total < tolerance:
            return True
            
        # Calcular coseno del ángulo entre direcciones
        start_mag = np.sqrt(start_dir[0]**2 + start_dir[1]**2) or 1e-10
        end_mag = np.sqrt(end_dir[0]**2 + end_dir[1]**2) or 1e-10
        
        cos_angle_start = (start_dir[0]*total_dir[0] + start_dir[1]*total_dir[1]) / (start_mag * length_total)
        cos_angle_end = (end_dir[0]*total_dir[0] + end_dir[1]*total_dir[1]) / (end_mag * length_total)
        
        # Si los ángulos son cercanos a 1, es casi recto
        return isclose(cos_angle_start, 1.0, abs_tol=tolerance) and isclose(cos_angle_end, 1.0, abs_tol=tolerance)
    
    # Para segmentos Bezier cuadráticos
    if isinstance(segment, QuadraticBezier):
        # Calcular vector de control
        control_dir = (segment.control.real - segment.start.real, 
                       segment.control.imag - segment.start.imag)
        
        # Calcular dirección general
        total_dir = (segment.end.real - segment.start.real, 
                     segment.end.imag - segment.start.imag)
        
        # Normalizar vectores
        length_total = np.sqrt(total_dir[0]**2 + total_dir[1]**2)
        if length_total < tolerance:
            return True
            
        control_mag = np.sqrt(control_dir[0]**2 + control_dir[1]**2) or 1e-10
        cos_angle = (control_dir[0]*total_dir[0] + control_dir[1]*total_dir[1]) / (control_mag * length_total)
        
        return isclose(cos_angle, 1.0, abs_tol=tolerance)
    
    # Para arcos, siempre se consideran curvas
    if isinstance(segment, Arc):
        return False
    
    # Por defecto, considerar como curva
    return False

# Función para calcular vectores normales
def calculate_normals(points):
    """Calcula vectores normales perpendiculares a la dirección de la pista"""
    normals = []
    n = len(points)
    
    for i in range(n):
        # Manejar casos especiales (primer y último punto)
        if i == 0:
            prev_point = points[-1] if n > 2 else points[0]
            next_point = points[1]
        elif i == n - 1:
            prev_point = points[i-1]
            next_point = points[0] if n > 2 else points[i]
        else:
            prev_point = points[i-1]
            next_point = points[i+1]
        
        # Vector de dirección
        dx = next_point[0] - prev_point[0]
        dy = next_point[1] - prev_point[1]
        
        # Calcular vector normal (perpendicular)
        length = np.sqrt(dx*dx + dy*dy)
        if length > 0:
            nx = -dy / length
            ny = dx / length
        else:
            nx, ny = 0, 0
        
        normals.append((nx, ny))
    
    return normals

# Parámetros
ancho_extra = 1        # Ancho total de la pista
num_points = 1000       # Número de puntos para la línea central
desired_length = 400    # Longitud deseada para la pista

# Cargar el archivo SVG
root = Tk()
root.withdraw()
svg_file = filedialog.askopenfilename(
    title="Selecciona el archivo SVG",
    filetypes=[("Archivos SVG", "*.svg"), ("Todos los archivos", "*.*")]
)
root.destroy()
if not svg_file:
    raise FileNotFoundError("No se ha seleccionado ningún archivo SVG.")

# Leer el archivo SVG y extraer los paths
paths, attributes = svg2paths(svg_file)

# Combinar todos los paths en uno solo
combined_path = Path()
for path in paths:
    combined_path.extend(path)

# Obtener lista de segmentos y sus longitudes
segments = list(combined_path)
segment_lengths = [seg.length() for seg in segments]
cumulative_segment_lengths = np.cumsum(segment_lengths)
total_length_orig = cumulative_segment_lengths[-1] if len(cumulative_segment_lengths) > 0 else 0

# Precalcular tipos de segmentos (0: recta, 1: curva)
segment_types = []
for seg in segments:
    if is_straight_segment(seg):
        segment_types.append(0)
    else:
        segment_types.append(1)

# Calcular factor de escala
scale_factor = desired_length / total_length_orig

# Muestrear puntos equidistantes en longitud de arco (en espacio original)
s_new_orig = np.linspace(0, total_length_orig, num_points)
x_central_orig = []
y_central_orig = []
curve_flags = []  # lista de tipos para cada punto (0 = recta, 1 = curva)

current_segment_index = 0
current_segment_start_length = 0.0

for s in s_new_orig:
    # Encontrar el segmento que contiene a s
    while (current_segment_index < len(cumulative_segment_lengths) and (s > cumulative_segment_lengths[current_segment_index])):
        current_segment_index += 1
        if current_segment_index < len(segments):
            current_segment_start_length = cumulative_segment_lengths[current_segment_index-1] if current_segment_index>0 else 0.0
        else:
            break
    
    if current_segment_index >= len(segments):
        # Último segmento
        segment = segments[-1]
        segment_type = segment_types[-1]
        # s_local en el último segmento
        s_local = s - (cumulative_segment_lengths[-2] if len(cumulative_segment_lengths) > 1 else 0)
    else:
        segment = segments[current_segment_index]
        segment_type = segment_types[current_segment_index]
        s_local = s - (cumulative_segment_lengths[current_segment_index-1] if current_segment_index>0 else 0)
    
    # Longitud del segmento actual
    seg_len = segment.length()
    # Parámetro t_local en el segmento (de 0 a 1)
    t_local = s_local / seg_len if seg_len > 0 else 0.0
    t_local = max(0, min(t_local, 1.0))  # Asegurar dentro de [0,1]
    
    # Evaluar el punto en el segmento
    pt = segment.point(t_local)
    x_central_orig.append(pt.real)
    y_central_orig.append(-pt.imag)  # Invertir eje Y para visualización
    curve_flags.append(segment_type)

# Convertir a arrays y escalar
x_central_orig = np.array(x_central_orig)
y_central_orig = np.array(y_central_orig)
x_central = x_central_orig * scale_factor
y_central = y_central_orig * scale_factor
curve_flags = np.array(curve_flags)

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
plt.plot(x_central, y_central, 'r-', linewidth=2, label="Línea central")
plt.plot(x_border1, y_border1, 'b-', linewidth=2, label="Borde exterior")
plt.plot(x_border2, y_border2, 'g-', linewidth=2, label="Borde interior")

# Resaltar segmentos curvos
curve_points = np.where(curve_flags == 1)
plt.scatter(x_central[curve_points], y_central[curve_points], color='orange', s=10, label="Segmentos curvos")

plt.axis("equal")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title(f"Pista de carreras escalada a {desired_length}m (Rectas: {np.sum(curve_flags==0)}, Curvas: {np.sum(curve_flags==1)})")
plt.legend()
plt.grid(True)
plt.show()

# Seleccionar ubicación para guardar CSV
root = Tk()
root.withdraw()
csv_path = filedialog.asksaveasfilename(
    title="Guardar archivo CSV",
    defaultextension=".csv",
    filetypes=[("Archivos CSV", "*.csv")]
)
root.destroy()

if csv_path:
    # Guardar en CSV con información de segmentos
    all_data = np.column_stack((x_central, y_central, x_border1, y_border1, x_border2, y_border2, curve_flags))
    np.savetxt(csv_path, all_data, delimiter=",", 
               header="x_central,y_central,x_border1,y_border1,x_border2,y_border2,is_curve", 
               comments="")
    print(f"Archivo CSV guardado en: {csv_path}")
    print(f"Puntos totales: {num_points}")
    print(f"Segmentos rectos: {np.sum(curve_flags==0)}")
    print(f"Segmentos curvos: {np.sum(curve_flags==1)}")
else:
    print("Operación cancelada por el usuario")
import numpy as np
import matplotlib.pyplot as plt
from svg.path import parse_path, Line, CubicBezier, QuadraticBezier, Arc
import xml.etree.ElementTree as ET
from tkinter import Tk, filedialog
import csv
import os
from math import isclose, sqrt

def is_straight_segment(segment, tolerance=0.01):
    """Determina si un segmento es recto basado en la curvatura"""
    if isinstance(segment, Line):
        return True
        
    if isinstance(segment, (CubicBezier, QuadraticBezier)):
        start = complex(segment.start)
        end = complex(segment.end)
        
        if isinstance(segment, CubicBezier):
            c1 = complex(segment.control1)
            c2 = complex(segment.control2)
            # Calcular desviación máxima de la línea recta
            t = np.linspace(0, 1, 10)
            points = (1-t)**3 * start + 3*(1-t)**2*t*c1 + 3*(1-t)*t**2*c2 + t**3*end
        else:  # QuadraticBezier
            c = complex(segment.control)
            t = np.linspace(0, 1, 10)
            points = (1-t)**2 * start + 2*(1-t)*t*c + t**2*end
        
        # Vector de dirección
        dir_vec = end - start
        length = abs(dir_vec)
        if length < 1e-5:
            return True
            
        # Calcular distancia máxima perpendicular
        max_distance = 0
        for p in points:
            vec = p - start
            # Proyección sobre la dirección
            parallel = np.dot([vec.real, vec.imag], [dir_vec.real, dir_vec.imag]) / length**2
            parallel_vec = parallel * dir_vec
            perp_vec = vec - parallel_vec
            distance = abs(perp_vec)
            if distance > max_distance:
                max_distance = distance
                
        # Si la desviación máxima es pequeña, es recta
        return max_distance < tolerance * length
        
    if isinstance(segment, Arc):
        return False
        
    return False

def calculate_normals(points, smoothing=3):
    """Calcula vectores normales con suavizado"""
    if not points:
        return []
        
    normals = []
    n = len(points)
    
    # Calcular direcciones preliminares
    for i in range(n):
        # Puntos adyacentes
        prev_idx = i-1 if i > 0 else n-1
        next_idx = i+1 if i < n-1 else 0
        
        # Vector de dirección
        dx = points[next_idx][0] - points[prev_idx][0]
        dy = points[next_idx][1] - points[prev_idx][1]
        
        # Calcular normal preliminar
        length = sqrt(dx*dx + dy*dy)
        if length > 1e-5:
            nx = -dy / length
            ny = dx / length
        else:
            nx, ny = 0, 0
        normals.append((nx, ny))
    
    # Suavizado
    if smoothing > 1 and normals:
        smoothed_normals = []
        for i in range(n):
            avg_nx, avg_ny = 0, 0
            count = 0
            start_j = max(0, i-smoothing)
            end_j = min(n, i+smoothing+1)
            for j in range(start_j, end_j):
                avg_nx += normals[j][0]
                avg_ny += normals[j][1]
                count += 1
            if count > 0:
                avg_nx /= count
                avg_ny /= count
            else:
                avg_nx, avg_ny = normals[i]
            
            # Renormalizar
            length = sqrt(avg_nx*avg_nx + avg_ny*avg_ny)
            if length > 1e-5:
                avg_nx /= length
                avg_ny /= length
            smoothed_normals.append((avg_nx, avg_ny))
        return smoothed_normals
    
    return normals

def sample_path(path, n_points=1000):
    """Muestrea un camino SVG con detección de curvatura mejorada"""
    points = []
    curve_flags = []
    
    if not path:
        return points, curve_flags
        
    total_length = path.length()
    if total_length <= 0:
        return points, curve_flags
    
    # Precalcular segmentos
    segments = list(path)
    if not segments:
        return points, curve_flags
        
    segment_lengths = [seg.length() for seg in segments]
    cumulative_lengths = np.cumsum(segment_lengths)
    total_length = cumulative_lengths[-1]
    
    # Detectar tipo de segmento
    segment_types = [0 if is_straight_segment(seg) else 1 for seg in segments]
    
    # Muestrear puntos
    for i in range(n_points):
        s = i * total_length / (n_points - 1)
        
        # Encontrar segmento
        seg_idx = 0
        while seg_idx < len(cumulative_lengths) and s > cumulative_lengths[seg_idx]:
            seg_idx += 1
        
        # Asegurar que no exceda el rango
        seg_idx = min(seg_idx, len(segments)-1)
        
        if seg_idx == 0:
            seg_start = 0
        else:
            seg_start = cumulative_lengths[seg_idx-1]
            
        seg = segments[seg_idx]
        seg_len = segment_lengths[seg_idx]
        
        t = (s - seg_start) / seg_len if seg_len > 0 else 0
        t = max(0, min(t, 1.0))
        pt = seg.point(t)
        points.append((pt.real, pt.imag))
        curve_flags.append(segment_types[seg_idx])
    
    return points, curve_flags

def calculate_total_length(points):
    """Calcula la longitud total de una lista de puntos"""
    total = 0.0
    if not points:
        return total
        
    for i in range(1, len(points)):
        x1, y1 = points[i-1]
        x2, y2 = points[i]
        dx = x2 - x1
        dy = y2 - y1
        total += sqrt(dx*dx + dy*dy)
    return total

def main():
    # Configurar interfaz
    root = Tk()
    root.withdraw()
    
    # Seleccionar SVG
    svg_path = filedialog.askopenfilename(
        title="Seleccionar archivo SVG",
        filetypes=[("Archivos SVG", "*.svg")]
    )
    if not svg_path:
        print("Operación cancelada")
        return
    
    try:
        # Parsear SVG
        tree = ET.parse(svg_path)
        root_elem = tree.getroot()
        
        # Buscar caminos
        namespaces = {'svg': 'http://www.w3.org/2000/svg'}
        paths = root_elem.findall('.//svg:path', namespaces)
        if not paths:
            raise ValueError("No se encontraron elementos <path> en el SVG")
        
        # Procesar primer camino
        main_path = parse_path(paths[0].get('d'))
        center_points_orig, curve_flags = sample_path(main_path, 1000)
        if not center_points_orig:
            raise ValueError("No se pudo muestrear el camino")
        
        # Calcular normales con geometría ORIGINAL
        normals = calculate_normals(center_points_orig, smoothing=5)
        
        # Generar bordes en espacio ORIGINAL
        track_width = 10.0  # Ancho de pista en unidades SVG
        inner_points_orig = []
        outer_points_orig = []
        for i, (x, y) in enumerate(center_points_orig):
            nx, ny = normals[i]
            inner_points_orig.append((x - track_width/2 * nx, y - track_width/2 * ny))
            outer_points_orig.append((x + track_width/2 * nx, y + track_width/2 * ny))
        
        # Calcular longitud total del circuito en unidades SVG
        total_original_length = calculate_total_length(center_points_orig)
        if total_original_length <= 0:
            raise ValueError("Longitud del circuito debe ser positiva")
        
        # Factor de escalado: deseamos un circuito de 4000 metros (4 km)
        desired_length = 4000.0  # en metros
        scale_factor = desired_length / total_original_length
        
        # Escalar TODOS los puntos
        center_points = [(x*scale_factor, y*scale_factor) for (x,y) in center_points_orig]
        inner_points = [(x*scale_factor, y*scale_factor) for (x,y) in inner_points_orig]
        outer_points = [(x*scale_factor, y*scale_factor) for (x,y) in outer_points_orig]
        
        # Seleccionar destino CSV
        csv_path = filedialog.asksaveasfilename(
            title="Guardar archivo CSV",
            defaultextension=".csv",
            filetypes=[("Archivos CSV", "*.csv")]
        )
        if not csv_path:
            print("Operación cancelada")
            return
        
        # Guardar CSV
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x_central', 'y_central', 'x_border1', 'y_border1', 'x_border2', 'y_border2', 'is_curve'])
            for i in range(len(center_points)):
                row = [
                    center_points[i][0], center_points[i][1],
                    inner_points[i][0], inner_points[i][1],
                    outer_points[i][0], outer_points[i][1],
                    curve_flags[i]
                ]
                writer.writerow(row)
        
        print(f"Archivo guardado: {csv_path}")
        print(f"Puntos: {len(center_points)}")
        print(f"Longitud original: {total_original_length:.2f} unidades SVG")
        print(f"Longitud escalada: {desired_length} metros")
        print(f"Factor de escalado: {scale_factor:.6f}")
        print(f"Rectas: {curve_flags.count(0)}")
        print(f"Curvas: {curve_flags.count(1)}")
        
        # Visualización rápida
        plt.figure(figsize=(10, 8))
        x_c = [p[0] for p in center_points]
        y_c = [p[1] for p in center_points]
        x_i = [p[0] for p in inner_points]
        y_i = [p[1] for p in inner_points]
        x_o = [p[0] for p in outer_points]
        y_o = [p[1] for p in outer_points]
        
        plt.plot(x_c, y_c, 'b-', alpha=0.5, label='Central')
        plt.plot(x_i, y_i, 'g-', alpha=0.5, label='Borde Interno')
        plt.plot(x_o, y_o, 'r-', alpha=0.5, label='Borde Externo')
        
        # Resaltar curvas
        curve_x = [x_c[i] for i in range(len(curve_flags)) if curve_flags[i] == 1]
        curve_y = [y_c[i] for i in range(len(curve_flags)) if curve_flags[i] == 1]
        plt.scatter(curve_x, curve_y, c='red', s=10, label='Curvas')
        
        plt.axis('equal')
        plt.legend()
        plt.title(os.path.basename(svg_path))
        plt.xlabel('X (metros)')
        plt.ylabel('Y (metros)')
        plt.grid(True)
        plt.show()
        
    except Exception as e:
        print(f"Error: {str(e)}")

if __name__ == "__main__":
    main()
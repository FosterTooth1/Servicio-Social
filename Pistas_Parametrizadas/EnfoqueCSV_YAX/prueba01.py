import tkinter as tk
from tkinter import filedialog
import xml.etree.ElementTree as ET
from svg.path import parse_path, Line, CubicBezier, QuadraticBezier, Arc
import csv
import numpy as np
from math import isclose

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

def offset_path_with_normals(center_points, normals, offset):
    """Desplaza puntos usando vectores normales"""
    inner_points = []
    outer_points = []
    
    for (x, y), (nx, ny) in zip(center_points, normals):
        inner_points.append((x - offset * nx, y - offset * ny))
        outer_points.append((x + offset * nx, y + offset * ny))
    
    return inner_points, outer_points

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
        cos_angle_start = (start_dir[0]*total_dir[0] + start_dir[1]*total_dir[1]) / (
            np.sqrt(start_dir[0]**2 + start_dir[1]**2) * length_total + 1e-10)
        cos_angle_end = (end_dir[0]*total_dir[0] + end_dir[1]*total_dir[1]) / (
            np.sqrt(end_dir[0]**2 + end_dir[1]**2) * length_total + 1e-10)
        
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
            
        # Calcular coseno del ángulo
        cos_angle = (control_dir[0]*total_dir[0] + control_dir[1]*total_dir[1]) / (
            np.sqrt(control_dir[0]**2 + control_dir[1]**2) * length_total + 1e-10)
        
        return isclose(cos_angle, 1.0, abs_tol=tolerance)
    
    # Para arcos, siempre se consideran curvas
    if isinstance(segment, Arc):
        return False
    
    # Por defecto, considerar como curva
    return False

def sample_path_with_curvature(path, n_points=1000):
    """Muestrea un camino SVG y detecta segmentos rectos vs curvos"""
    points = []
    is_curve = []  # 0 = recta, 1 = curva
    total_length = path.length()
    
    # Precalcular segmentos y sus longitudes
    segments = list(path)
    segment_lengths = [seg.length() for seg in segments]
    segment_types = [0 if is_straight_segment(seg) else 1 for seg in segments]
    
    # Acumulador de longitud
    current_length = 0
    
    for i in range(n_points):
        fraction = i / (n_points - 1)
        pt = path.point(fraction)
        points.append((pt.real, pt.imag))
        
        # Determinar en qué segmento está este punto
        target_length = fraction * total_length
        seg_index = 0
        accum_length = 0
        
        for idx, seg_len in enumerate(segment_lengths):
            accum_length += seg_len
            if accum_length >= target_length:
                seg_index = idx
                break
        
        # Usar el tipo del segmento encontrado
        is_curve.append(segment_types[seg_index])
    
    return points, is_curve

def main():
    # Configurar interfaz gráfica
    root = tk.Tk()
    root.withdraw()
    
    # Seleccionar archivo SVG
    print("Selecciona el archivo SVG del circuito")
    svg_path = filedialog.askopenfilename(
        title="Seleccionar SVG",
        filetypes=[("Archivos SVG", "*.svg")]
    )
    
    if not svg_path:
        print("No se seleccionó ningún archivo")
        return
    
    try:
        # Parsear el SVG
        tree = ET.parse(svg_path)
        root_elem = tree.getroot()
        
        # Buscar caminos principales
        namespaces = {'svg': 'http://www.w3.org/2000/svg'}
        paths = root_elem.findall('.//svg:path', namespaces)
        
        if not paths:
            raise ValueError("El SVG no contiene caminos (<path>)")
        
        # Procesar el primer camino
        main_path = parse_path(paths[0].get('d'))
        center_points, curve_flags = sample_path_with_curvature(main_path)
        
        # Calcular vectores normales
        normals = calculate_normals(center_points)
        
        # Crear bordes con desplazamiento perpendicular
        TRACK_WIDTH = 10.0  # Ancho de la pista (ajustar según necesidad)
        inner_points, outer_points = offset_path_with_normals(center_points, normals, TRACK_WIDTH/2)
        
        # Seleccionar ubicación para guardar CSV
        print("Selecciona dónde guardar el archivo CSV")
        csv_path = filedialog.asksaveasfilename(
            title="Guardar CSV",
            filetypes=[("Archivos CSV", "*.csv")],
            defaultextension=".csv"
        )
        
        if not csv_path:
            print("No se seleccionó ubicación para guardar")
            return
        
        # Escribir CSV con columna adicional para curva/recta
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x_central', 'y_central', 'x_border1', 'y_border1', 
                            'x_border2', 'y_border2', 'is_curve'])
            
            for i in range(len(center_points)):
                row = [
                    center_points[i][0], center_points[i][1],
                    inner_points[i][0], inner_points[i][1],
                    outer_points[i][0], outer_points[i][1],
                    curve_flags[i]
                ]
                writer.writerow(row)
        
        print(f"Archivo CSV generado exitosamente en: {csv_path}")
        print(f"Puntos procesados: {len(center_points)}")
        print(f"Ancho de pista utilizado: {TRACK_WIDTH} unidades")
        print(f"Segmentos rectos: {curve_flags.count(0)}, Segmentos curvos: {curve_flags.count(1)}")
        
    except Exception as e:
        print(f"Error: {str(e)}")
        print("Recomendaciones:")
        print("- Asegúrate que el SVG contenga al menos un camino continuo")
        print("- Si los bordes no se ven correctamente, ajusta el valor de TRACK_WIDTH en el código")

if __name__ == "__main__":
    main()
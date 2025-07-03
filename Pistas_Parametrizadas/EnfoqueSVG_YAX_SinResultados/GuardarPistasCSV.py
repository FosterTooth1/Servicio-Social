import os
import csv
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import re
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# ——— Parámetros de segmentación y offset ———
default_robot_width_m = 0.30
scale = 1.0
offset = (default_robot_width_m / 2) * scale
curvature_threshold = 0.001
window_size = 15 # Must be odd
epsilon = 1e-5  # Tolerancia para coincidencia de puntos

# ——— Parámetros de reescalado ———
desired_length = 600  # longitud deseada de la pista
num_points = 800  # número total deseado de puntos en la línea central tras remuestreo

COMMANDS = set('MmLlCcZz')
TOKEN_RE = re.compile(r'[MmLlCcZz]|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?')

def parse_path_d(d):
    tokens, i = TOKEN_RE.findall(d), 0
    cur = np.zeros(2)
    start = cur.copy()
    pts = [] # List to collect individual points as NumPy arrays
    while i < len(tokens):
        cmd = tokens[i]
        if cmd in ('M', 'm'):
            is_rel = cmd == 'm'
            i += 1
            x, y = map(float, tokens[i:i + 2])
            i += 2
            cur = cur + [x, y] if is_rel else np.array([x, y])
            start = cur.copy()
            pts.append(cur.copy())
        elif cmd in ('L', 'l'):
            is_rel = cmd == 'l'
            i += 1
            while i + 1 < len(tokens) and tokens[i] not in COMMANDS:
                x, y = map(float, tokens[i:i + 2])
                i += 2
                cur = cur + [x, y] if is_rel else np.array([x, y])
                pts.append(cur.copy())
        elif cmd in ('C', 'c'):
            is_rel = cmd == 'c'
            i += 1
            while i + 5 < len(tokens) and tokens[i] not in COMMANDS:
                c = list(map(float, tokens[i:i + 6]))
                i += 6
                p1_ctrl = cur + c[0:2] if is_rel else np.array(c[0:2])
                p2_ctrl = cur + c[2:4] if is_rel else np.array(c[2:4])
                p3_end = cur + c[4:6] if is_rel else np.array(c[4:6])
                
                # Dynamic number of Bezier points based on segment length, or minimum
                # This initial parsing is still quite dense to preserve detail for curvature
                approx_length = np.linalg.norm(p1_ctrl - cur) + np.linalg.norm(p2_ctrl - p1_ctrl) + np.linalg.norm(p3_end - p2_ctrl)
                num_bezier_points = max(5, int(approx_length / 2)) # Adjust divisor for point density
                
                for t in np.linspace(0, 1, num_bezier_points):
                    point = (1 - t)**3 * cur + \
                            3 * (1 - t)**2 * t * p1_ctrl + \
                            3 * (1 - t) * t**2 * p2_ctrl + \
                            t**3 * p3_end
                    pts.append(point)
                cur = p3_end.copy()
        elif cmd in ('Z', 'z'):
            pts.append(start.copy())
            cur, start = start.copy(), start.copy()
            i += 1
        else:
            raise ValueError(f"Token inesperado: {cmd}")
    
    if not pts: # If no points were parsed, return empty array
        return np.array([])
    return np.vstack(pts)

def compute_curvature(pts):
    κ = np.zeros(len(pts))
    if len(pts) < 3:
        return κ
    
    dx = np.gradient(pts[:, 0])
    dy = np.gradient(pts[:, 1])
    d2x = np.gradient(dx)
    d2y = np.gradient(dy)

    numerator = dx * d2y - dy * d2x
    denominator = (dx**2 + dy**2)**1.5
    
    κ = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator!=0)

    return κ

def detect_segments(pts, κ, thr):
    is_st = np.abs(κ) < thr
    segs_raw, idxs, typ = [], [], None
    for i, st in enumerate(is_st):
        t = 'straight' if st else 'curve'
        if t != typ:
            if idxs:
                segs_raw.append((typ, idxs.copy()))
            idxs, typ = [i], t
        else:
            idxs.append(i)
    if idxs:
        segs_raw.append((typ, idxs))
    return segs_raw

def ensure_continuity(segs_data, original_pts):
    """Ensures continuity between segments and closes the loop if needed."""
    if not segs_data:
        return []

    processed_segments = []
    
    for typ, indices in segs_data:
        if not indices: continue
        
        start_idx = indices[0]
        end_idx = indices[-1] + 1 
        
        if end_idx > len(original_pts):
            end_idx = len(original_pts)
        
        segment_points = original_pts[start_idx : end_idx]
        
        if len(segment_points) < 1: continue

        if typ == 'straight':
            processed_segments.append({
                'type': 'straight',
                'p0': segment_points[0].tolist(),
                'p1': segment_points[-1].tolist(),
                'points': segment_points.tolist()
            })
        else: # curve
            processed_segments.append({
                'type': 'curve',
                'points': segment_points.tolist()
            })

    final_segments = []
    if not processed_segments:
        return []

    final_segments.append(processed_segments[0])

    for i in range(len(processed_segments) - 1):
        current_seg_end = np.array(processed_segments[i]['points'][-1])
        next_seg_start = np.array(processed_segments[i+1]['points'][0])

        if not np.allclose(current_seg_end, next_seg_start, atol=epsilon):
            # Add a connecting straight segment if there's a gap
            final_segments.append({
                'type': 'straight',
                'p0': current_seg_end.tolist(),
                'p1': next_seg_start.tolist(),
                'points': [current_seg_end.tolist(), next_seg_start.tolist()]
            })
        final_segments.append(processed_segments[i+1])

    # Ensure the last point connects to the first point to close the loop
    if final_segments and len(final_segments) > 1:
        first_point_track = np.array(final_segments[0]['points'][0])
        last_point_track = np.array(final_segments[-1]['points'][-1])
        if not np.allclose(first_point_track, last_point_track, atol=epsilon):
            final_segments.append({
                'type': 'straight',
                'p0': last_point_track.tolist(),
                'p1': first_point_track.tolist(),
                'points': [last_point_track.tolist(), first_point_track.tolist()]
            })

    return final_segments


def segment_track(svg_path):
    tree = ET.parse(svg_path)
    root = tree.getroot()
    ns = {'svg': root.tag.split('}')[0].strip('{')}
    
    # Use a list to collect all points from all paths
    all_raw_points_list = [] 

    # Iterate over all path elements and combine their parsed points
    for e in root.findall('.//svg:path', ns):
        path_pts_array = parse_path_d(e.get('d')) # parse_path_d now returns a numpy array
        
        if path_pts_array.size > 0: # Ensure path_pts is not empty
            if all_raw_points_list: # If there are already points, connect
                # Check if the last point of the previous path is close to the first point
                # of the current path. If not, add a connecting straight segment.
                last_pt_prev_path = all_raw_points_list[-1]
                first_pt_curr_path = path_pts_array[0]
                
                if not np.allclose(last_pt_prev_path, first_pt_curr_path, atol=epsilon):
                    # Generate a few points for the connecting line, avoiding too many
                    num_connecting_points = max(2, int(np.linalg.norm(first_pt_curr_path - last_pt_prev_path) / 0.05)) # Dynamic based on distance
                    connecting_line = np.linspace(last_pt_prev_path, first_pt_curr_path, num=num_connecting_points)
                    all_raw_points_list.extend(connecting_line[1:].tolist()) # Add points as lists, exclude first to avoid duplication
                
                all_raw_points_list.extend(path_pts_array.tolist()) # Add current path points as lists
            else: # First path, just add points
                all_raw_points_list.extend(path_pts_array.tolist())

    if not all_raw_points_list:
        print("No se encontraron puntos de camino válidos en el SVG.")
        return []

    # Convert the list of lists/arrays to a single NumPy array for further processing
    all_combined_pts = np.array(all_raw_points_list)

    # Apply Savitzky-Golay filter to curvature for smoothing
    κ = compute_curvature(all_combined_pts)
    # Ensure window_size is valid for savgol_filter
    if len(κ) >= window_size and window_size % 2 == 1:
        κ_smooth = savgol_filter(κ, window_size, 3)
    else:
        # Fallback if window_size is too large or not odd
        print(f"Advertencia: window_size ({window_size}) es inválido para el número de puntos ({len(κ)}) o no es impar. No se aplicará el filtro Savitzky-Golay.")
        κ_smooth = κ # Use raw curvature

    raw_segments_detected = detect_segments(all_combined_pts, κ_smooth, curvature_threshold)
    
    segments = ensure_continuity(raw_segments_detected, all_combined_pts)
    
    return segments

def rescale_and_sample(segs):
    """Remuestrea y escala los segmentos manteniendo el tipo de segmento"""
    if not segs:
        return [], [], [], [], [], [], []

    # 1) Calcular longitud total de la pista
    total_length = 0
    for seg in segs:
        pts = np.array(seg['points']) 
        if len(pts) > 1:
            dx = np.diff(pts[:, 0])
            dy = np.diff(pts[:, 1])
            total_length += np.sum(np.sqrt(dx**2 + dy**2))

    if total_length == 0:
        print("La longitud total de la pista es cero. No se puede reescalar.")
        return [], [], [], [], [], [], []

    # 2) Factor de escala
    factor = desired_length / total_length

    # 3) Procesar cada segmento
    all_xc, all_yc, all_b1x, all_b1y, all_b2x, all_b2y, all_types = [], [], [], [], [], [], []

    for seg_idx, seg in enumerate(segs):
        pts = np.array(seg['points']) * factor # Scale points for all segment types
        
        if len(pts) < 2:
            continue 

        # Calculate segment length after scaling
        dx_seg = np.diff(pts[:, 0])
        dy_seg = np.diff(pts[:, 1])
        scaled_seg_length = np.sum(np.sqrt(dx_seg**2 + dy_seg**2))

        # Determine number of points for this segment, proportional to its length
        # Ensure a minimum number of points for very short segments
        # The total number of points `num_points` is distributed proportionally.
        num_seg_points = max(2, int(num_points * (scaled_seg_length / desired_length)))
        
        # Arc length parametrization for uniform sampling
        distances = np.concatenate(([0], np.cumsum(np.sqrt(dx_seg**2 + dy_seg**2))))
        
        if distances[-1] == 0: 
            xc_segment = [pts[0,0]]
            yc_segment = [pts[0,1]]
        else:
            fx = interp1d(distances, pts[:, 0], kind='linear', fill_value="extrapolate")
            fy = interp1d(distances, pts[:, 1], kind='linear', fill_value="extrapolate")
            s_new = np.linspace(0, distances[-1], num_seg_points)
            xc_segment = fx(s_new)
            yc_segment = fy(s_new)

        # Calculate normals for the resampled points
        if len(xc_segment) > 1:
            dx_resampled = np.gradient(xc_segment)
            dy_resampled = np.gradient(yc_segment)
            mag_resampled = np.sqrt(dx_resampled**2 + dy_resampled**2)
            mag_resampled[mag_resampled == 0] = 1e-8 # Avoid division by zero
            nx = -dy_resampled / mag_resampled
            ny = dx_resampled / mag_resampled
        else: # Single point segment
            nx = np.array([0.0])
            ny = np.array([0.0])

        # Calculate borders
        b1x = xc_segment + offset * nx
        b1y = yc_segment + offset * ny
        b2x = xc_segment - offset * nx
        b2y = yc_segment - offset * ny

        # Assign segment type (0 for straight, 1 for curve)
        seg_type_val = 0 if seg['type'] == 'straight' else 1
        segment_type = np.full(len(xc_segment), seg_type_val)
        
        all_xc.extend(xc_segment)
        all_yc.extend(yc_segment)
        all_b1x.extend(b1x)
        all_b1y.extend(b1y)
        all_b2x.extend(b2x)
        all_b2y.extend(b2y)
        all_types.extend(segment_type)
            
    return all_xc, all_yc, all_b1x, all_b1y, all_b2x, all_b2y, all_types

def export_csv(svg_path):
    # Crear carpeta para CSV
    os.makedirs('output_csv', exist_ok=True)
    
    # Segmentar la pista primero
    segs = segment_track(svg_path)
    if not segs:
        print("No se encontraron segmentos en el SVG o el procesamiento falló.")
        return
        
    # Remuestrear y escalar con información de segmentos
    xc, yc, b1x, b1y, b2x, b2y, segment_types = rescale_and_sample(segs)
    
    if not xc: # Check if points were generated after resampling
        print("No se generaron puntos después del remuestreo y escalado.")
        return

    # Guardar CSV
    csvf = os.path.join('output_csv', 
                        os.path.splitext(os.path.basename(svg_path))[0] + '.csv')
    with open(csvf, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x_central', 'y_central', 'x_border1', 'y_border1', 
                    'x_border2', 'y_border2', 'segment_type'])
        for i in range(len(xc)):
            w.writerow([
                f'{xc[i]:.18e}', f'{yc[i]:.18e}',
                f'{b1x[i]:.18e}', f'{b1y[i]:.18e}',
                f'{b2x[i]:.18e}', f'{b2y[i]:.18e}',
                int(segment_types[i])
            ])
    print("CSV guardado en:", csvf)

    # Opcional: mostrar gráfica con colores por tipo de segmento
    plt.figure(figsize=(10, 8))
    
    # Convertir a arrays para facilitar el manejo
    xc = np.array(xc)
    yc = np.array(yc)
    b1x = np.array(b1x)
    b1y = np.array(b1y)
    b2x = np.array(b2x)
    b2y = np.array(b2y)
    segment_types = np.array(segment_types)
    
    # Graficar rectas (tipo 0)
    straight_mask = segment_types == 0
    if np.any(straight_mask):
        plt.plot(xc[straight_mask], yc[straight_mask], 'bo', markersize=2, label='Centro (recta)')
        plt.plot(b1x[straight_mask], b1y[straight_mask], 'r-', linewidth=1, label='Borde1 (recta)')
        plt.plot(b2x[straight_mask], b2y[straight_mask], 'g-', linewidth=1, label='Borde2 (recta)')
    
    # Graficar curvas (tipo 1)
    curve_mask = segment_types == 1
    if np.any(curve_mask):
        plt.plot(xc[curve_mask], yc[curve_mask], 'mo', markersize=2, label='Centro (curva)')
        plt.plot(b1x[curve_mask], b1y[curve_mask], 'c-', linewidth=1, label='Borde1 (curva)')
        plt.plot(b2x[curve_mask], b2y[curve_mask], 'y-', linewidth=1, label='Borde2 (curva)')
    
    plt.title("Pista segmentada con rectas y curvas")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    root = Tk()
    root.withdraw()
    fn = askopenfilename(title="Selecciona SVG", filetypes=[("SVG", "*.svg")])
    if fn:
        export_csv(fn)
    else:
        print("No se seleccionó archivo.")
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
window_size = 15
epsilon = 1e-5  # Tolerancia para coincidencia de puntos

# ——— Parámetros de reescalado ———
desired_length = 400  # longitud deseada de la pista
num_points = 600  # número de puntos en la línea central tras remuestreo

COMMANDS = set('MmLlCcZz')
TOKEN_RE = re.compile(r'[MmLlCcZz]|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?')

def parse_path_d(d):
    tokens, i = TOKEN_RE.findall(d), 0
    cur = np.zeros(2)
    start = cur.copy()
    pts = []
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
                p1 = cur + c[0:2] if is_rel else np.array(c[0:2])
                p2 = cur + c[2:4] if is_rel else np.array(c[2:4])
                p3 = cur + c[4:6] if is_rel else np.array(c[4:6])
                for t in np.linspace(0, 1, 20):
                    pts.append(((1 - t) ** 3) * cur
                               + 3 * ((1 - t) ** 2) * t * p1
                               + 3 * (1 - t) * (t ** 2) * p2
                               + (t ** 3) * p3)
                cur = p3.copy()
        elif cmd in ('Z', 'z'):
            pts.append(start.copy())
            cur, start = start.copy(), start.copy()
            i += 1
        else:
            raise ValueError(f"Token inesperado: {cmd}")
    return np.vstack(pts)

def compute_curvature(pts):
    κ = np.zeros(len(pts))
    for i in range(1, len(pts) - 1):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]
        dx, dy = (x2 - x0) / 2, (y2 - y0) / 2
        d2x, d2y = x2 - 2 * x1 + x0, y2 - 2 * y1 + y0
        denom = (dx * dx + dy * dy) ** 1.5
        κ[i] = (dx * d2y - dy * d2x) / denom if denom else 0
    return κ

def detect_segments(pts, κ, thr):
    is_st = np.abs(κ) < thr
    segs, idxs, typ = [], [], None
    for i, st in enumerate(is_st):
        t = 'straight' if st else 'curve'
        if t != typ:
            if idxs:
                segs.append((typ, idxs.copy()))
            idxs, typ = [i], t
        else:
            idxs.append(i)
    if idxs:
        segs.append((typ, idxs))
    return segs

def ensure_continuity(segs):
    """Garantiza continuidad entre segmentos corrigiendo pequeños huecos"""
    if not segs:
        return segs

    # Asegurar que el circuito sea cerrado
    first_point = None
    last_point = None

    for i, seg in enumerate(segs):
        if seg['type'] == 'straight':
            p0 = np.array(seg['p0'])
            p1 = np.array(seg['p1'])
            if i == 0:
                first_point = p0
            last_point = p1
        else:
            ctr = np.array(seg['controls'])
            if i == 0:
                first_point = ctr[0]
            last_point = ctr[-1]

    # Si el circuito no está cerrado, añadir un segmento de conexión
    if not np.allclose(first_point, last_point, atol=epsilon):
        segs.append({
            'type': 'straight',
            'p0': last_point.tolist(),
            'p1': first_point.tolist()
        })

    # Corregir discontinuidades entre segmentos consecutivos
    for i in range(len(segs) - 1):
        current_seg = segs[i]
        next_seg = segs[i + 1]

        if current_seg['type'] == 'straight':
            current_end = np.array(current_seg['p1'])
        else:
            current_end = np.array(current_seg['controls'][-1])

        if next_seg['type'] == 'straight':
            next_start = np.array(next_seg['p0'])
        else:
            next_start = np.array(next_seg['controls'][0])

        # Si hay una discontinuidad, ajustar el punto de inicio del siguiente segmento
        if not np.allclose(current_end, next_start, atol=epsilon):
            if next_seg['type'] == 'straight':
                next_seg['p0'] = current_end.tolist()
            else:
                next_seg['controls'][0] = current_end.tolist()

    return segs

def segment_track(svg_path):
    tree = ET.parse(svg_path)
    root = tree.getroot()
    ns = {'svg': root.tag.split('}')[0].strip('{')}
    raw = []
    for e in root.findall('.//svg:path', ns):
        ctr = parse_path_d(e.get('d'))
        κ = savgol_filter(compute_curvature(ctr), window_size, 3)
        raw += [(typ, ctr[idxs]) for typ, idxs in detect_segments(ctr, κ, curvature_threshold)]
        break  # Solo procesamos el primer path

    if not raw:
        return []

    # Reordenar si empieza con recta
    if raw[0][0] == 'straight':
        raw = raw[1:] + raw[:1]

    recomb = []
    for typ, pts in raw:
        if recomb and recomb[-1][0] == typ:
            recomb[-1] = (typ, np.vstack([recomb[-1][1], pts]))
        else:
            recomb.append((typ, pts))

    segments = []
    last_end = None

    for typ, pts in recomb:
        # Si hay un hueco con el segmento anterior, crear segmento recto de conexión
        if last_end is not None and not np.allclose(pts[0], last_end, atol=epsilon):
            segments.append({
                'type': 'straight',
                'p0': last_end.tolist(),
                'p1': pts[0].tolist()
            })

        if typ == 'straight':
            segment = {
                'type': 'straight',
                'p0': pts[0].tolist(),
                'p1': pts[-1].tolist()
            }
            segments.append(segment)
            last_end = pts[-1].copy()
        else:
            segment = {
                'type': 'curve',
                'points': pts.tolist()
            }
            segments.append(segment)
            last_end = pts[-1].copy()

    # Garantizar continuidad en todo el circuito
    segments = ensure_continuity(segments)

    return segments

def rescale_and_sample(segs):
    """Remuestrea y escala los segmentos manteniendo el tipo de segmento"""
    if not segs:
        return [], [], [], [], []

    # 1) Calcular longitud total de la pista
    total_length = 0
    for seg in segs:
        if seg['type'] == 'straight':
            p0 = np.array(seg['p0'])
            p1 = np.array(seg['p1'])
            total_length += np.linalg.norm(p1 - p0)
        else:  # curve
            pts = np.array(seg['points'])
            dx = np.diff(pts[:, 0])
            dy = np.diff(pts[:, 1])
            total_length += np.sqrt(dx * dx + dy * dy).sum()

    # 2) Factor de escala
    factor = desired_length / total_length

    # 3) Procesar cada segmento
    all_xc, all_yc, all_b1x, all_b1y, all_b2x, all_b2y, all_types = [], [], [], [], [], [], []

    for seg in segs:
        if seg['type'] == 'straight':
            p0 = np.array(seg['p0'])
            p1 = np.array(seg['p1'])
            
            # Escalar puntos
            p0 *= factor
            p1 *= factor
            
            # Crear línea recta con puntos intermedios
            num_seg_points = max(2, int(np.linalg.norm(p1 - p0) / total_length * num_points))
            t = np.linspace(0, 1, num_seg_points)
            xc = p0[0] + t * (p1[0] - p0[0])
            yc = p0[1] + t * (p1[1] - p0[1])
            
            # Calcular normales
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            mag = np.sqrt(dx * dx + dy * dy)
            if mag > 0:
                nx, ny = -dy / mag, dx / mag
            else:
                nx, ny = 0, 0
            
            # Calcular bordes
            b1x = xc + offset * nx
            b1y = yc + offset * ny
            b2x = xc - offset * nx
            b2y = yc - offset * ny
            
            # Tipo (0 = recta)
            seg_type = np.zeros(len(xc))
            
        else:  # curve
            pts = np.array(seg['points'])
            # Escalar puntos
            pts *= factor
            
            # Calcular longitud del segmento
            dx = np.diff(pts[:, 0])
            dy = np.diff(pts[:, 1])
            seg_length = np.sqrt(dx * dx + dy * dy).sum()
            
            # Número de puntos proporcional a la longitud
            num_seg_points = max(10, int(seg_length / total_length * num_points))
            
            # Parametrización por longitud de arco
            ds = np.concatenate([[0], np.cumsum(np.sqrt(np.diff(pts[:, 0])**2 + np.diff(pts[:, 1])**2))])
            fx = interp1d(ds, pts[:, 0], kind='linear')
            fy = interp1d(ds, pts[:, 1], kind='linear')
            s_new = np.linspace(0, ds[-1], num_seg_points)
            xc = fx(s_new)
            yc = fy(s_new)
            
            # Calcular normales
            dx = np.gradient(xc)
            dy = np.gradient(yc)
            mag = np.sqrt(dx * dx + dy * dy)
            mag[mag == 0] = 1e-8
            nx, ny = -dy / mag, dx / mag
            
            # Calcular bordes
            b1x = xc + offset * nx
            b1y = yc + offset * ny
            b2x = xc - offset * nx
            b2y = yc - offset * ny
            
            # Tipo (1 = curva)
            seg_type = np.ones(len(xc))
        
        # Agregar a los arrays globales
        all_xc.extend(xc)
        all_yc.extend(yc)
        all_b1x.extend(b1x)
        all_b1y.extend(b1y)
        all_b2x.extend(b2x)
        all_b2y.extend(b2y)
        all_types.extend(seg_type)
    
    return all_xc, all_yc, all_b1x, all_b1y, all_b2x, all_b2y, all_types

def export_csv(svg_path):
    # Crear carpeta para CSV
    os.makedirs('output_csv', exist_ok=True)
    
    # Segmentar la pista primero
    segs = segment_track(svg_path)
    if not segs:
        print("No se encontraron segmentos en el SVG.")
        return
    
    # Remuestrear y escalar con información de segmentos
    xc, yc, b1x, b1y, b2x, b2y, segment_types = rescale_and_sample(segs)
    
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
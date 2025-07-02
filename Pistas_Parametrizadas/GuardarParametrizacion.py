import os
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import re
from scipy.signal import savgol_filter
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# Parámetros
default_robot_width_m = 0.30
offset = default_robot_width_m / 2
curvature_threshold = 0.001
window_size = 15
bezier_to_line_angle = 10  # grados
epsilon = 1e-5  # Tolerancia para coincidencia de puntos

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
            rel, i = (cmd == 'm'), i + 1
            x, y = map(float, tokens[i:i + 2])
            i += 2
            cur = cur + [x, y] if rel else np.array([x, y])
            start = cur.copy()
            pts.append(cur.copy())
        elif cmd in ('L', 'l'):
            rel, i = (cmd == 'l'), i + 1
            while i + 1 < len(tokens) and tokens[i] not in COMMANDS:
                x, y = map(float, tokens[i:i + 2])
                i += 2
                cur = cur + [x, y] if rel else np.array([x, y])
                pts.append(cur.copy())
        elif cmd in ('C', 'c'):
            rel, i = (cmd == 'c'), i + 1
            while i + 5 < len(tokens) and tokens[i] not in COMMANDS:
                c = list(map(float, tokens[i:i + 6]))
                i += 6
                p1 = cur + c[0:2] if rel else np.array(c[0:2])
                p2 = cur + c[2:4] if rel else np.array(c[2:4])
                p3 = cur + c[4:6] if rel else np.array(c[4:6])
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

def fit_cubic_bezier(pts):
    n = len(pts)
    return np.vstack([pts[0], pts[n // 3], pts[2 * n // 3], pts[-1]])

def angle_between(u, v):
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu * nv == 0:
        return 0
    return np.degrees(np.arccos(np.clip(np.dot(u / nu, v / nv), -1, 1)))

def offset_curves(pts):
    tang = np.gradient(pts, axis=0)
    norms = np.linalg.norm(tang, axis=1, keepdims=True)
    norms[norms == 0] = 1
    tang /= norms
    normals = np.column_stack([-tang[:, 1], tang[:, 0]])
    return pts - normals * offset, pts + normals * offset

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
            bez = fit_cubic_bezier(pts)
            v1 = bez[1] - bez[0]
            v2 = bez[-1] - bez[-2]
            angle = angle_between(v1, v2)
            
            if angle < bezier_to_line_angle:
                segment = {
                    'type': 'straight',
                    'p0': bez[0].tolist(),
                    'p1': bez[-1].tolist()
                }
                segments.append(segment)
                last_end = bez[-1].copy()
            else:
                segment = {
                    'type': 'curve',
                    'controls': bez.tolist()
                }
                segments.append(segment)
                last_end = bez[-1].copy()
    
    # Garantizar continuidad en todo el circuito
    segments = ensure_continuity(segments)
    
    return segments

def load_and_plot(svg_path):
    segs = segment_track(svg_path)
    if not segs:
        print("No se encontraron segmentos en el SVG.")
        return

    # Diccionario para controlar las etiquetas
    seen = {
        'straight_center': False,
        'straight_inner': False,
        'straight_outer': False,
        'curve_inner': False,
        'curve_outer': False,
        'curve_points': False
    }

    plt.figure()
    for seg in segs:
        if seg['type'] == 'straight':
            a = np.array(seg['p0'])
            b = np.array(seg['p1'])
            pts = np.vstack([a, b])
            inn, out = offset_curves(pts)

            # Línea central (sólida)
            plt.plot(pts[:, 0], pts[:, 1], 'b-', lw=2,
                     label='Centro recta' if not seen['straight_center'] else '_nolegend_')
            seen['straight_center'] = True

            # Cotas (discontinuas)
            plt.plot(inn[:, 0], inn[:, 1], 'b--', lw=1,
                     label='Cota int. recta' if not seen['straight_inner'] else '_nolegend_')
            plt.plot(out[:, 0], out[:, 1], 'b--', lw=1,
                     label='Cota ext. recta' if not seen['straight_outer'] else '_nolegend_')
            seen['straight_inner'] = seen['straight_outer'] = True

        else:  # curve
            ctr = np.array(seg['controls'])
            t = np.linspace(0, 1, 200)
            curve = np.array([(1 - ti) ** 3 * ctr[0]
                             + 3 * (1 - ti) ** 2 * ti * ctr[1]
                             + 3 * (1 - ti) * ti ** 2 * ctr[2]
                             + ti ** 3 * ctr[3] for ti in t])

            # Puntos de muestra (no la curva completa)
            idxs = np.linspace(0, len(curve) - 1, 15).astype(int)
            sample_pts = curve[idxs]
            plt.plot(sample_pts[:, 0], sample_pts[:, 1], 'o',
                     color='purple', markersize=4,
                     label='Puntos curva' if not seen['curve_points'] else '_nolegend_')
            seen['curve_points'] = True

            # Cotas (discontinuas)
            inn, out = offset_curves(curve)
            plt.plot(inn[:, 0], inn[:, 1], 'r--', lw=1,
                     label='Cota int. curva' if not seen['curve_inner'] else '_nolegend_')
            plt.plot(out[:, 0], out[:, 1], 'r--', lw=1,
                     label='Cota ext. curva' if not seen['curve_outer'] else '_nolegend_')
            seen['curve_inner'] = seen['curve_outer'] = True

    # Configuración del gráfico
    plt.title("Pista con cotas, puntos Bézier y segmentos conectados")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

    # Guardar SVG (sin cambios)
    out_dir = 'param_svg'
    os.makedirs(out_dir, exist_ok=True)
    all_pts = []
    for seg in segs:
        if seg['type'] == 'straight':
            all_pts += [seg['p0'], seg['p1']]
        else:
            all_pts += seg['controls']
    all_pts = np.vstack(all_pts)
    xmin, ymin = all_pts.min(axis=0)
    xmax, ymax = all_pts.max(axis=0)
    svg = ET.Element('svg', xmlns="http://www.w3.org/2000/svg",
                     attrib={'viewBox': f"{xmin:.3f} {ymin:.3f} {xmax - xmin:.3f} {ymax - ymin:.3f}"})

    def mkpath(coords, color, dash=''):
        d = 'M ' + ' L '.join(f"{x:.3f},{y:.3f}" for x, y in coords)
        a = {'d': d, 'stroke': color, 'fill': 'none'}
        if dash:
            a['stroke-dasharray'] = dash
        return ET.Element('path', a)

    for seg in segs:
        if seg['type'] == 'straight':
            coords = [seg['p0'], seg['p1']]
            svg.append(mkpath(coords, '#0000ff', '5,5'))
            inn, out = offset_curves(np.array(coords))
            svg.append(mkpath(inn.tolist(), '#0000ff', ''))
            svg.append(mkpath(out.tolist(), '#0000ff', ''))
        else:
            ctr = np.array(seg['controls'])
            t = np.linspace(0, 1, 200)
            curve = np.array([(1 - ti) ** 3 * ctr[0]
                             + 3 * (1 - ti) ** 2 * ti * ctr[1]
                             + 3 * (1 - ti) * ti ** 2 * ctr[2]
                             + ti ** 3 * ctr[3] for ti in t])
            svg.append(mkpath(curve.tolist(), '#ff0000', ''))
            inn, out = offset_curves(curve)
            svg.append(mkpath(inn.tolist(), '#ff0000', ''))
            svg.append(mkpath(out.tolist(), '#ff0000', ''))

    name = os.path.splitext(os.path.basename(svg_path))[0]
    out_f = os.path.join(out_dir, f"{name}_param.svg")
    ET.ElementTree(svg).write(out_f)
    print("SVG guardado en:", out_f)


if __name__ == "__main__":
    root = Tk()
    root.withdraw()
    fn = askopenfilename(title="Selecciona SVG", filetypes=[("SVG", "*.svg")])
    if fn:
        load_and_plot(fn)
    else:
        print("No se seleccionó archivo.")
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import re
from scipy.signal import savgol_filter
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# Parámetros
default_robot_width_m = 0.30
scale = 1.0
offset = (default_robot_width_m / 2) * scale
curvature_threshold = 0.001  # Umbral para detectar rectas
window_size = 15            # Tamaño del filtro de curvatura

# Parser SVG
COMMANDS = set('MmLlCcZz')
TOKEN_RE = re.compile(r'[MmLlCcZz]|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?')

def parse_path_d(d):
    tokens = TOKEN_RE.findall(d)
    i = 0
    cur = np.zeros(2, dtype=float)
    start = cur.copy()
    points = []

    while i < len(tokens):
        cmd = tokens[i]

        if cmd in ('M', 'm'):
            is_rel = (cmd == 'm')
            i += 1
            x, y = float(tokens[i]), float(tokens[i+1])
            cur = cur + [x, y] if is_rel else np.array([x, y], dtype=float)
            start = cur.copy()
            points.append(cur.copy())
            i += 2
            while i+1 < len(tokens) and tokens[i] not in COMMANDS:
                x, y = float(tokens[i]), float(tokens[i+1])
                cur = cur + [x, y] if is_rel else np.array([x, y], dtype=float)
                points.append(cur.copy())
                i += 2

        elif cmd in ('L', 'l'):
            is_rel = (cmd == 'l')
            i += 1
            while i+1 < len(tokens) and tokens[i] not in COMMANDS:
                x, y = float(tokens[i]), float(tokens[i+1])
                cur = cur + [x, y] if is_rel else np.array([x, y], dtype=float)
                points.append(cur.copy())
                i += 2

        elif cmd in ('C', 'c'):
            is_rel = (cmd == 'c')
            i += 1
            while i+5 < len(tokens) and tokens[i] not in COMMANDS:
                coords = list(map(float, tokens[i:i+6]))
                p1 = cur + coords[0:2] if is_rel else np.array(coords[0:2], dtype=float)
                p2 = cur + coords[2:4] if is_rel else np.array(coords[2:4], dtype=float)
                p3 = cur + coords[4:6] if is_rel else np.array(coords[4:6], dtype=float)
                for t in np.linspace(0, 1, 20):
                    pt = ((1-t)**3) * cur + 3 * ((1-t)**2) * t * p1 + 3 * (1-t) * (t**2) * p2 + (t**3) * p3
                    points.append(pt)
                cur = p3.copy()
                i += 6

        elif cmd in ('Z', 'z'):
            points.append(start.copy())
            cur = start.copy()
            i += 1

        else:
            raise ValueError(f"Token inesperado: {cmd!r}")

    return np.vstack(points)


def compute_curvature(pts):
    n = len(pts)
    curvature = np.zeros(n)
    for i in range(1, n-1):
        x_prev, y_prev = pts[i-1]
        x_curr, y_curr = pts[i]
        x_next, y_next = pts[i+1]

        dx_dt = (x_next - x_prev) / 2.0
        dy_dt = (y_next - y_prev) / 2.0
        d2x_dt2 = x_next - 2*x_curr + x_prev
        d2y_dt2 = y_next - 2*y_curr + y_prev
        denom = (dx_dt**2 + dy_dt**2)**1.5
        curvature[i] = ((dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / denom) if denom != 0 else 0
    return curvature


def detect_segments(pts, curvature, threshold):
    is_straight = np.abs(curvature) < threshold
    segments = []
    current_pts = []
    current_type = None
    for i, pt in enumerate(pts):
        seg_type = 'straight' if is_straight[i] else 'curve'
        if seg_type != current_type:
            if current_pts:
                segments.append((current_type, np.vstack(current_pts)))
            current_pts = [pt]
            current_type = seg_type
        else:
            current_pts.append(pt)
    if current_pts:
        segments.append((current_type, np.vstack(current_pts)))
    return segments


def load_and_plot(svg_path):
    tree = ET.parse(svg_path)
    root = tree.getroot()
    ns = {'svg': root.tag.split('}')[0].strip('{')}

    for elem in root.findall('.//svg:path', ns):
        center_pts = parse_path_d(elem.get('d'))
        curvature = compute_curvature(center_pts)
        curvature = savgol_filter(curvature, window_size, 3)
        segments = detect_segments(center_pts, curvature, curvature_threshold)

        plt.figure()
        straight_plotted = False
        curve_plotted = False

        for seg_type, seg_pts in segments:
            if seg_type == 'straight':
                lbl = 'Recta' if not straight_plotted else '_nolegend_'
                plt.plot(seg_pts[:,0], seg_pts[:,1], 'b-', lw=2, label=lbl)
                straight_plotted = True
            else:
                lbl = 'Curva' if not curve_plotted else '_nolegend_'
                if seg_pts.shape[0] >= 4:
                    t = np.linspace(0, 1, len(seg_pts))
                    bezier = np.array([((1 - t[i])**3) * seg_pts[0] +
                                       3 * ((1 - t[i])**2) * t[i] * seg_pts[1] +
                                       3 * (1 - t[i]) * (t[i]**2) * seg_pts[2] +
                                       (t[i]**3) * seg_pts[3]
                                       for i in range(len(t))])
                    plt.plot(bezier[:,0], bezier[:,1], 'r-', lw=2, label=lbl)
                else:
                    plt.plot(seg_pts[:,0], seg_pts[:,1], 'r-', lw=2, label=lbl)
                curve_plotted = True

        plt.title("Pista seccionada: rectas (azul) y curvas (rojo)")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()
        return

    raise RuntimeError("No encontré ningún <path> válido en el SVG.")

if __name__ == '__main__':
    root = Tk(); root.withdraw()
    svg_file = askopenfilename(title="Selecciona SVG", filetypes=[("SVG", "*.svg")])
    if svg_file:
        load_and_plot(svg_file)
    else:
        print("No se seleccionó archivo.")

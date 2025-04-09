#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Parametrización y segmentación de una pista SVG en el plano x,y,
dividiendo la pista en rectas según análisis de curvatura.
Requiere: numpy, matplotlib
"""

import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import re
import os

# ——— Parser de path SVG sin dependencias externas ———
import numpy as np
import re

# Reconoce comandos M/m, L/l, C/c, Z/z, y números (p.ej. -123, 45.67, 8.9e-3)
TOKEN_RE = re.compile(r'[MmLlCcZz]|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?')

def parse_path_d(d):
    """
    Parsea el atributo d de un <path> SVG,
    soporta absolutos/relativos y devuelve segmentos con coordenadas absolutas.
    """
    tokens = TOKEN_RE.findall(d)
    i = 0
    cur = np.zeros(2)
    start = np.zeros(2)
    segments = []

    while i < len(tokens):
        tok = tokens[i]

        # MOVE
        if tok in ('M','m'):
            is_rel = (tok == 'm')
            x = float(tokens[i+1]); y = float(tokens[i+2])
            pt = cur + np.array([x,y]) if is_rel else np.array([x,y])
            cur = pt.copy()
            start = pt.copy()
            i += 3

        # LINE
        elif tok in ('L','l'):
            is_rel = (tok == 'l')
            x = float(tokens[i+1]); y = float(tokens[i+2])
            new = cur + np.array([x,y]) if is_rel else np.array([x,y])
            segments.append(('L', cur.copy(), new.copy()))
            cur = new.copy()
            i += 3

        # CUBIC BEZIER
        elif tok in ('C','c'):
            is_rel = (tok == 'c')
            coords = list(map(float, tokens[i+1:i+7]))
            if is_rel:
                p1 = cur + np.array(coords[0:2])
                p2 = cur + np.array(coords[2:4])
                p3 = cur + np.array(coords[4:6])
            else:
                p1 = np.array(coords[0:2])
                p2 = np.array(coords[2:4])
                p3 = np.array(coords[4:6])
            segments.append(('C', cur.copy(), p1.copy(), p2.copy(), p3.copy()))
            cur = p3.copy()
            i += 7

        # CLOSE PATH
        elif tok in ('Z','z'):
            segments.append(('L', cur.copy(), start.copy()))
            cur = start.copy()
            i += 1

        else:
            # Si aparece algo que no sea comando, es señal de que hubo un fallo
            raise ValueError(f"Token inesperado: {tok!r}")

    return segments


def load_segments(svg_path, path_id=None):
    tree = ET.parse(svg_path)
    root = tree.getroot()
    ns = {'svg': root.tag.split('}')[0].strip('{')}
    for elem in root.findall('.//svg:path', ns):
        if path_id and elem.get('id') == path_id:
            return parse_path_d(elem.get('d'))
    first = root.find('.//svg:path', ns)
    return parse_path_d(first.get('d'))

# ——— Parametrización de la curva ———
def approx_cubic(p0,p1,p2,p3,t):
    return (1-t)**3*p0 + 3*(1-t)**2*t*p1 + 3*(1-t)*t**2*p2 + t**3*p3

def sample_track(segments, n_points=1000):
    # Calcular longitudes de cada segmento
    lengths = []
    for seg in segments:
        if seg[0]=='L':
            lengths.append(np.linalg.norm(seg[2]-seg[1]))
        else:
            pts = [approx_cubic(seg[1],seg[2],seg[3],seg[4],tt) for tt in np.linspace(0,1,20)]
            lengths.append(sum(np.linalg.norm(pts[i+1]-pts[i]) for i in range(len(pts)-1)))
    total_L = sum(lengths)

    # Muestreo equidistante en s
    s_vals = np.linspace(0,total_L,n_points)
    xs = np.zeros(n_points); ys = np.zeros(n_points)
    seg_start_s = 0
    idx = 0
    for i, s in enumerate(s_vals):
        # Avanza hasta el segmento correcto
        while idx < len(segments)-1 and s > seg_start_s + lengths[idx]:
            seg_start_s += lengths[idx]
            idx += 1
        seg = segments[idx]
        local_s = s - seg_start_s
        if seg[0]=='L':
            P0,P1 = seg[1], seg[2]
            t_local = local_s/lengths[idx]
            pt = (1-t_local)*P0 + t_local*P1
        else:
            t_est = local_s/lengths[idx]
            pt = approx_cubic(seg[1],seg[2],seg[3],seg[4], t_est)
        xs[i], ys[i] = pt
    return s_vals, xs, ys

# ——— Cálculo de curvatura ———
def compute_curvature(xs, ys):
    dx = np.gradient(xs); dy = np.gradient(ys)
    ddx = np.gradient(dx); ddy = np.gradient(dy)
    return (dx*ddy - dy*ddx) / (dx*dx + dy*dy)**1.5

# ——— Filtrado de curvatura (media móvil) ———
def smooth(x, window_len=21):
    w = np.ones(window_len)/window_len
    return np.convolve(x, w, mode='same')

# ——— Identificación de segmentos rectos ———
def find_straights(k, min_length=50, percentile=5):
    abs_k = np.abs(k)
    thresh = np.percentile(abs_k, percentile)
    mask = abs_k < thresh
    straights = []
    start = None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if (not m or i==len(mask)-1) and start is not None:
            if i-start >= min_length:
                straights.append((start, i))
            start = None
    return straights

# ——— Gráficas ———
def plot_track(xs, ys, straights):
    plt.figure()
    plt.plot(xs, ys, label='Centerline')
    for idx,(i,j) in enumerate(straights):
        plt.plot(xs[i:j], ys[i:j], linestyle='--',
                 label='Recta' if idx==0 else "")
    plt.axis('equal')
    plt.title('Pista y segmentos rectos')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()

def plot_curvature(s, k_smooth, straights):
    plt.figure()
    plt.plot(s, k_smooth, label='Curvatura')
    for idx,(i,j) in enumerate(straights):
        plt.plot(s[i:j], k_smooth[i:j], linestyle='--',
                 label='Recta' if idx==0 else "")
    plt.title('Curvatura vs distancia')
    plt.xlabel('s (unidades)')
    plt.ylabel('curvatura')
    plt.legend()
    plt.show()

# ——— Main ———
if __name__=='__main__':
    svg_file = r"C:\Users\anyax\Desktop\SS CODIGO\Servicio-Social\Pistas con Direccion\Autódromo_Hermanos_Rodríguez_2015 (Bandera y Direccion).svg"
    if not os.path.exists(svg_file):
        raise FileNotFoundError(f"No encontré el SVG en:\n{svg_file}")

    segments = load_segments(svg_file)
    s, xs, ys = sample_track(segments, n_points=2000)
    k = compute_curvature(xs, ys)
    k_smooth = smooth(k, window_len=31)
    straights = find_straights(k_smooth, min_length=100, percentile=5)

    # Mostrar gráficas
    plot_track(xs, ys, straights)
    plot_curvature(s, k_smooth, straights)

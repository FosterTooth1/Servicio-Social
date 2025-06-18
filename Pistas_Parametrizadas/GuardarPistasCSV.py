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
curvature_threshold   = 0.001
window_size           = 15

# ——— Parámetros de reescalado ———
desired_length = 400    # longitud deseada de la pista
num_points     = 600    # número de puntos en la línea central tras remuestreo

COMMANDS = set('MmLlCcZz')
TOKEN_RE = re.compile(r'[MmLlCcZz]|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?')

def parse_path_d(d):
    tokens,i = TOKEN_RE.findall(d),0
    cur = np.zeros(2); start=cur.copy(); pts=[]
    while i < len(tokens):
        cmd = tokens[i]
        if cmd in ('M','m'):
            is_rel = cmd=='m'; i+=1
            x,y = map(float, tokens[i:i+2])
            cur = cur+[x,y] if is_rel else np.array([x,y])
            start,pts,i = cur.copy(), pts+[cur.copy()], i+2
        elif cmd in ('L','l'):
            is_rel = cmd=='l'; i+=1
            while i+1<len(tokens) and tokens[i] not in COMMANDS:
                x,y = map(float, tokens[i:i+2])
                cur = cur+[x,y] if is_rel else np.array([x,y])
                pts.append(cur.copy()); i+=2
        elif cmd in ('C','c'):
            is_rel = cmd=='c'; i+=1
            while i+5<len(tokens) and tokens[i] not in COMMANDS:
                c = list(map(float, tokens[i:i+6]))
                p1 = cur+c[0:2] if is_rel else np.array(c[0:2])
                p2 = cur+c[2:4] if is_rel else np.array(c[2:4])
                p3 = cur+c[4:6] if is_rel else np.array(c[4:6])
                for t in np.linspace(0,1,20):
                    pts.append(((1-t)**3)*cur
                               +3*((1-t)**2)*t*p1
                               +3*(1-t)*(t**2)*p2
                               + (t**3)*p3)
                cur,i = p3.copy(), i+6
        elif cmd in ('Z','z'):
            pts.append(start.copy()); cur,start,i = start.copy(), start.copy(), i+1
        else:
            raise ValueError(f"Token inesperado: {cmd}")
    return np.vstack(pts)

def rescale_and_sample(svg_path):
    # 1) Extraer todos los puntos de todos los <path> en el SVG
    tree = ET.parse(svg_path)
    root = tree.getroot()
    ns   = {'svg': root.tag.split('}')[0].strip('{')}
    all_pts = []
    for e in root.findall('.//svg:path', ns):
        d = e.get('d')
        pts = parse_path_d(d)
        all_pts.append(pts)
    all_pts = np.vstack(all_pts)

    # 2) Calcular longitud total
    dx = np.diff(all_pts[:,0])
    dy = np.diff(all_pts[:,1])
    total_length = np.sqrt(dx*dx + dy*dy).sum()

    # 3) Escala uniforme
    factor = desired_length / total_length
    xs = all_pts[:,0] * factor
    ys = all_pts[:,1] * factor

    # 4) Parametrización por longitud de arco
    ds = np.concatenate([[0], np.cumsum(np.sqrt(np.diff(xs)**2 + np.diff(ys)**2))])
    fx = interp1d(ds, xs, kind='linear')
    fy = interp1d(ds, ys, kind='linear')
    s_new = np.linspace(0, ds[-1], num_points)
    xc, yc = fx(s_new), fy(s_new)

    # 5) Cálculo de normales
    dxn = np.gradient(xc)
    dyn = np.gradient(yc)
    mag = np.sqrt(dxn*dxn + dyn*dyn)
    mag[mag==0] = 1e-8
    nx, ny = -dyn/mag, dxn/mag

    return xc, yc, nx, ny

def export_csv(svg_path):
    # carpeta solo CSV
    os.makedirs('output_csv', exist_ok=True)

    # remuestrear y escalar
    xc, yc, nx, ny = rescale_and_sample(svg_path)
    # Borde1 externo, Borde2 interno
    b1x, b1y = xc + offset*nx, yc + offset*ny
    b2x, b2y = xc - offset*nx, yc - offset*ny

    # Guardar CSV
    csvf = os.path.join('output_csv',
        os.path.splitext(os.path.basename(svg_path))[0] + '.csv')
    with open(csvf, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x_central','y_central','x_border1','y_border1','x_border2','y_border2'])
        for x0,y0,x1,y1,x2,y2 in zip(xc,yc,b1x,b1y,b2x,b2y):
            w.writerow([f'{x0:.18e}', f'{y0:.18e}',
                        f'{x1:.18e}', f'{y1:.18e}',
                        f'{x2:.18e}', f'{y2:.18e}'])
    print("CSV guardado en:", csvf)

    # Opcional: mostrar gráfica
    plt.figure(figsize=(6,6))
    plt.plot(xc, yc, 'k-', linewidth=2, label='Centro remuestreado')
    plt.plot(b1x,b1y,'r--',linewidth=1,label='Borde externo')
    plt.plot(b2x,b2y,'b--',linewidth=1,label='Borde interno')
    plt.title("Pista remuestreada y escalada")
    plt.xlabel("X"); plt.ylabel("Y")
    plt.axis('equal'); plt.legend(); plt.grid(True)
    plt.show()

if __name__=='__main__':
    root = Tk(); root.withdraw()
    fn = askopenfilename(title="Selecciona SVG", filetypes=[("SVG","*.svg")])
    if fn:
        export_csv(fn)
    else:
        print("No se seleccionó archivo.")

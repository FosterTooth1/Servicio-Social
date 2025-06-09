#FUNCIONA POR EL MOMENTO CON LAS PISTAS: AUTODROMO HERMANOS RODRIGUEZ,NURBURING, SUZUKA
#FALTAN AJUSTAR DETALLES: AUISTIN CIRCUIT
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
curvature_threshold    = 0.001
window_size           = 15
min_segment_length    = 0.01
angle_threshold_deg   = 30
bezier_to_line_angle  = 10  # grados

COMMANDS = set('MmLlCcZz')
TOKEN_RE = re.compile(r'[MmLlCcZz]|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?')

def parse_path_d(d):
    tokens = TOKEN_RE.findall(d); i=0
    cur = np.zeros(2); start = cur.copy(); pts=[]
    while i < len(tokens):
        cmd = tokens[i]
        if cmd in ('M','m'):
            is_rel = cmd=='m'; i+=1
            x,y = map(float, tokens[i:i+2])
            cur = cur+[x,y] if is_rel else np.array([x,y])
            start=cur.copy(); pts.append(cur.copy()); i+=2
            while i+1<len(tokens) and tokens[i] not in COMMANDS:
                x,y = map(float, tokens[i:i+2])
                cur = cur+[x,y] if is_rel else np.array([x,y])
                pts.append(cur.copy()); i+=2
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
                cur = p3.copy(); i+=6
        elif cmd in ('Z','z'):
            pts.append(start.copy()); cur=start.copy(); i+=1
        else:
            raise ValueError(f"Token inesperado: {cmd!r}")
    return np.vstack(pts)

def compute_curvature(pts):
    n=len(pts); κ=np.zeros(n)
    for i in range(1,n-1):
        x0,y0 = pts[i-1]; x1,y1 = pts[i]; x2,y2 = pts[i+1]
        dx,dy = (x2-x0)/2,(y2-y0)/2
        d2x,d2y = x2-2*x1+x0, y2-2*y1+y0
        denom = (dx*dx+dy*dy)**1.5
        κ[i] = (dx*d2y-dy*d2x)/denom if denom else 0
    return κ

def detect_segments(pts, κ, thr):
    is_st = np.abs(κ)<thr
    segs, idxs, typ = [], [], None
    for i in range(len(pts)):
        t = 'straight' if is_st[i] else 'curve'
        if t!=typ:
            if idxs: segs.append((typ, idxs.copy()))
            idxs,typ = [i], t
        else:
            idxs.append(i)
    if idxs: segs.append((typ, idxs))
    return segs

def length_of(pts):
    return np.linalg.norm(np.diff(pts,axis=0),axis=1).sum()

def fit_cubic_bezier(pts):
    n=len(pts)
    return np.vstack([pts[0], pts[n//3], pts[2*n//3], pts[-1]])

def split_bezier(ctrls):
    P0,P1,P2,P3 = ctrls
    L1,L2,L3 = (P0+P1)/2,(P1+P2)/2,(P2+P3)/2
    M1,M2    = (L1+L2)/2, (L2+L3)/2
    Mid      = (M1+M2)/2
    return (np.vstack([P0,L1,M1,Mid]), np.vstack([Mid,M2,L3,P3]))

def angle_between(u, v):
    nu = np.linalg.norm(u)
    nv = np.linalg.norm(v)
    if nu == 0 or nv == 0:
        return 0.0
    cu = u / nu
    cv = v / nv
    return np.degrees(np.arccos(np.clip(np.dot(cu, cv), -1.0, 1.0)))

def compute_offset_curves(pts, offset):
    tangents = np.gradient(pts, axis=0)
    norms = np.linalg.norm(tangents, axis=1, keepdims=True)
    norms[norms == 0] = 1
    tangents /= norms
    normals = np.column_stack([-tangents[:,1], tangents[:,0]])
    inner = pts - normals * offset
    outer = pts + normals * offset
    return inner, outer

def segment_track(svg_path):
    tree=ET.parse(svg_path); root=tree.getroot()
    ns={'svg':root.tag.split('}')[0].strip('{')}
    raw=[]
    for e in root.findall('.//svg:path', ns):
        ctr=parse_path_d(e.get('d'))
        κ = savgol_filter(compute_curvature(ctr), window_size, 3)
        for typ,idxs in detect_segments(ctr,κ,curvature_threshold):
            raw.append((typ, ctr[idxs]))
        break
    merged = raw.copy()
    if merged and merged[0][0]=='straight':
        merged = merged[1:]+merged[:1]
    recomb=[]
    for typ,pts in merged:
        if recomb and recomb[-1][0]==typ:
            recomb[-1]=(typ, np.vstack([recomb[-1][1],pts]))
        else:
            recomb.append((typ,pts))

    segments=[]; last_end=None
    for typ,pts in recomb:
        if last_end is not None and np.linalg.norm(pts[0] - last_end) > 1e-6:
            # insertar recta para conectar hueco
            segments.append({'type':'straight','p0':last_end.tolist(),'p1':pts[0].tolist()})
        if typ=='straight':
            segments.append({'type':'straight','p0':pts[0].tolist(),'p1':pts[-1].tolist()})
            last_end = pts[-1]
        else:
            bez = fit_cubic_bezier(pts)
            v1 = bez[1] - bez[0]
            v2 = bez[-1] - bez[-2]
            angle = angle_between(v1, v2)
            if angle < bezier_to_line_angle:
                segments.append({'type':'straight','p0':bez[0].tolist(),'p1':bez[-1].tolist()})
                last_end = bez[-1]
                continue
            segments.append({'type':'curve','controls':bez.tolist()})
            last_end = bez[-1]
    return segments

def load_and_plot(svg_path):
    segs = segment_track(svg_path)

    plt.figure()
    seen = {
        ('straight','center'): False,
        ('straight','inner'):  False,
        ('straight','outer'):  False,
        ('curve','inner'):     False,
        ('curve','outer'):     False,
        'curve_points':        False
    }

    for seg in segs:
        if seg['type']=='straight':
            pts = np.vstack([seg['p0'], seg['p1']])
            inner, outer = compute_offset_curves(pts, offset)
            plt.plot(pts[:,0], pts[:,1], 'b-', lw=2,
                     label='Centro recta' if not seen[('straight','center')] else '_nolegend_')
            plt.plot(inner[:,0], inner[:,1], 'b--', lw=1,
                     label='Cota int. recta' if not seen[('straight','inner')] else '_nolegend_')
            plt.plot(outer[:,0], outer[:,1], 'b--', lw=1,
                     label='Cota ext. recta' if not seen[('straight','outer')] else '_nolegend_')
            seen[('straight','center')] = seen[('straight','inner')] = seen[('straight','outer')] = True

        else:
            ctr = np.array(seg['controls'])
            t_vals = np.linspace(0, 1, 200)
            curve = np.array([
                (1-t)**3*ctr[0] + 3*(1-t)**2*t*ctr[1] + 3*(1-t)*t**2*ctr[2] + t**3*ctr[3]
                for t in t_vals
            ])
            tang = np.gradient(curve, axis=0)
            norms = np.linalg.norm(tang, axis=1, keepdims=True)
            norms[norms==0] = 1
            tang /= norms
            normals = np.column_stack([-tang[:,1], tang[:,0]])
            inner = curve - normals*offset
            outer = curve + normals*offset
            idxs = np.linspace(0, len(curve)-1, 15).astype(int)
            sample_pts = curve[idxs]
            plt.plot(sample_pts[:,0], sample_pts[:,1], 'o', color='purple', markersize=4,
                     label='Puntos curva' if not seen['curve_points'] else '_nolegend_')
            seen['curve_points'] = True
            plt.plot(inner[:,0], inner[:,1], 'r--', lw=1,
                     label='Cota int. curva' if not seen[('curve','inner')] else '_nolegend_')
            plt.plot(outer[:,0], outer[:,1], 'r--', lw=1,
                     label='Cota ext. curva' if not seen[('curve','outer')] else '_nolegend_')
            seen[('curve','inner')] = seen[('curve','outer')] = True

    plt.title("Pista con cotas, puntos Bézier y segmentos conectados")
    plt.xlabel("x"); plt.ylabel("y")
    plt.legend()
    plt.grid(True); plt.axis('equal')
    plt.show()

if __name__=='__main__':
    root=Tk(); root.withdraw()
    fn=askopenfilename(title="Selecciona SVG", filetypes=[("SVG","*.svg")])
    if fn:
        load_and_plot(fn)
    else:
        print("No se seleccionó archivo.")
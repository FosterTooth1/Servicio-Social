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
curvature_threshold    = 0.001    # Umbral curvatura
window_size           = 15       # Savitzky–Golay
min_segment_length    = 0.01     # Longitud mínima
angle_threshold_deg   = 30       # Umbral de ángulo para fusionar

COMMANDS = set('MmLlCcZz')
TOKEN_RE = re.compile(r'[MmLlCcZz]|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?')

def parse_path_d(d):
    tokens = TOKEN_RE.findall(d); i=0
    cur = np.zeros(2); start = cur.copy(); pts=[]
    while i < len(tokens):
        cmd = tokens[i]
        # — mismo parser que antes —
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
    return (np.vstack([P0,L1,M1,Mid]),
            np.vstack([Mid,M2,L3,P3]))

def angle_between(u,v):
    cu = u/np.linalg.norm(u); cv = v/np.linalg.norm(v)
    ang = np.degrees(np.arccos(np.clip(np.dot(cu,cv),-1,1)))
    return ang

def segment_track(svg_path):
    # 1) parse+detect
    tree=ET.parse(svg_path); root=tree.getroot()
    ns={'svg':root.tag.split('}')[0].strip('{')}
    raw=[]
    for e in root.findall('.//svg:path', ns):
        ctr=parse_path_d(e.get('d'))
        κ = savgol_filter(compute_curvature(ctr), window_size, 3)
        for typ,idxs in detect_segments(ctr,κ,curvature_threshold):
            raw.append((typ, ctr[idxs]))
        break

    # 2) fusionar muy cortos intermedios entre curvas si ángulo pequeño
    merged=[]
    i=0
    while i<len(raw):
        typ,pts = raw[i]
        if (typ=='straight'
         and length_of(pts)<min_segment_length
         and i>0 and i<len(raw)-1
         and raw[i-1][0]=='curve' and raw[i+1][0]=='curve'):
            # calcular ángulo
            prev_pts = raw[i-1][1]; next_pts = raw[i+1][1]
            v1 = prev_pts[-1]-prev_pts[-2]
            v2 = next_pts[1]-next_pts[0]
            if angle_between(v1,v2)<angle_threshold_deg:
                # fusionar prev+this+next en una curva
                all_pts = np.vstack([prev_pts, pts, next_pts])
                merged.pop()  # quita la curva previa
                merged.append(('curve', all_pts))
                i+=2  # saltar siguiente curva
                continue
        merged.append((typ,pts))
        i+=1

    # 3) asegurar alternancia empezando en curve
    if merged and merged[0][0]=='straight':
        merged = merged[1:]+merged[:1]
    recomb=[]
    for typ,pts in merged:
        if recomb and recomb[-1][0]==typ:
            recomb[-1]=(typ, np.vstack([recomb[-1][1],pts]))
        else:
            recomb.append((typ,pts))

    # 4) construir con snap endpoints y split de curvas largas
    segments=[]; last_end=None
    for typ,pts in recomb:
        if typ=='straight':
            A = last_end if last_end is not None else pts[0]
            B = pts[-1]
            segments.append({'type':'straight','p0':A.tolist(),'p1':B.tolist()})
            last_end = B
        else:
            bez = fit_cubic_bezier(pts)
            # split si largo > recta anterior
            prev = segments[-1] if segments else None
            if prev and prev['type']=='straight':
                Ls = np.linalg.norm(np.array(prev['p1'])-np.array(prev['p0']))
                Lc = length_of(pts)
                if Lc>Ls:
                    b1,b2 = split_bezier(bez)
                    b1[0]= last_end
                    b2[0]= b1[-1]
                    segments.append({'type':'curve','controls':b1.tolist()})
                    segments.append({'type':'curve','controls':b2.tolist()})
                    last_end = b2[-1]
                    continue
            bez[0] = last_end if last_end is not None else bez[0]
            segments.append({'type':'curve','controls':bez.tolist()})
            last_end= bez[-1]

    return segments

def load_and_plot(svg_path):
    segs = segment_track(svg_path)
    plt.figure(); s_pl=False; c_pl=False
    for seg in segs:
        if seg['type']=='straight':
            x0,y0=seg['p0']; x1,y1=seg['p1']
            lbl='Recta' if not s_pl else '_nolegend_'
            plt.plot([x0,x1],[y0,y1],'b-',lw=2,label=lbl)
            s_pl=True
        else:
            ctr=np.array(seg['controls'])
            t=np.linspace(0,1,50)
            bez=np.array([
                (1-ti)**3*ctr[0]
              +3*(1-ti)**2*ti*ctr[1]
              +3*(1-ti)*ti**2*ctr[2]
              +ti**3*ctr[3]
                for ti in t
            ])
            lbl='Curva' if not c_pl else '_nolegend_'
            plt.plot(bez[:,0],bez[:,1],'r-',lw=2,label=lbl)
            c_pl=True

    plt.title("Pista: sin gaps, curvas unidas")
    plt.xlabel("x"); plt.ylabel("y")
    plt.legend(); plt.grid(True); plt.axis('equal')
    plt.show()

if __name__=='__main__':
    root=Tk(); root.withdraw()
    fn=askopenfilename(title="Selecciona SVG", filetypes=[("SVG","*.svg")])
    if fn:
        load_and_plot(fn)
    else:
        print("No se seleccionó archivo.")

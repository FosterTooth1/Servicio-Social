import os
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import re
from scipy.signal import savgol_filter
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# Parámetros
default_robot_width_m  = 0.30
scale                  = 1.0
offset                 = (default_robot_width_m / 2) * scale
curvature_threshold    = 0.001
window_size            = 15
bezier_to_line_angle   = 10  # grados

COMMANDS = set('MmLlCcZz')
TOKEN_RE  = re.compile(r'[MmLlCcZz]|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?')

def parse_path_d(d):
    tokens,i = TOKEN_RE.findall(d),0
    cur = np.zeros(2); start=cur.copy(); pts=[]
    while i < len(tokens):
        cmd = tokens[i]
        if cmd in ('M','m'):
            is_rel = cmd=='m'; i+=1
            x,y = map(float, tokens[i:i+2]); i+=2
            cur = cur+[x,y] if is_rel else np.array([x,y])
            start, pts = cur.copy(), pts+[cur.copy()]
        elif cmd in ('L','l'):
            is_rel = cmd=='l'; i+=1
            while i+1<len(tokens) and tokens[i] not in COMMANDS:
                x,y = map(float, tokens[i:i+2]); i+=2
                cur = cur+[x,y] if is_rel else np.array([x,y])
                pts.append(cur.copy())
        elif cmd in ('C','c'):
            is_rel = cmd=='c'; i+=1
            while i+5<len(tokens) and tokens[i] not in COMMANDS:
                c = list(map(float, tokens[i:i+6])); i+=6
                p1 = cur+c[0:2] if is_rel else np.array(c[0:2])
                p2 = cur+c[2:4] if is_rel else np.array(c[2:4])
                p3 = cur+c[4:6] if is_rel else np.array(c[4:6])
                for t in np.linspace(0,1,20):
                    pts.append(((1-t)**3)*cur
                               +3*((1-t)**2)*t*p1
                               +3*(1-t)*(t**2)*p2
                               + (t**3)*p3)
                cur = p3.copy()
        elif cmd in ('Z','z'):
            pts.append(start.copy()); cur,start,i = start.copy(), start.copy(), i+1
        else:
            raise ValueError(f"Token inesperado: {cmd}")
    return np.vstack(pts)

def compute_curvature(pts):
    κ = np.zeros(len(pts))
    for i in range(1,len(pts)-1):
        x0,y0 = pts[i-1]; x1,y1 = pts[i]; x2,y2 = pts[i+1]
        dx,dy = (x2-x0)/2,(y2-y0)/2
        d2x,d2y = x2-2*x1+x0, y2-2*y1+y0
        denom = (dx*dx+dy*dy)**1.5
        κ[i] = (dx*d2y-dy*d2x)/denom if denom else 0
    return κ

def detect_segments(pts, κ, thr):
    is_st = np.abs(κ)<thr
    segs, idxs, typ = [], [], None
    for i,st in enumerate(is_st):
        t = 'straight' if st else 'curve'
        if t!=typ:
            if idxs: segs.append((typ, idxs.copy()))
            idxs,typ = [i], t
        else:
            idxs.append(i)
    if idxs: segs.append((typ, idxs))
    return segs

def fit_cubic_bezier(pts):
    n = len(pts)
    return np.vstack([pts[0], pts[n//3], pts[2*n//3], pts[-1]])

def angle_between(u, v):
    nu,nv = np.linalg.norm(u),np.linalg.norm(v)
    if nu*nv==0: return 0.0
    return np.degrees(np.arccos(np.clip(np.dot(u/nu, v/nv), -1,1)))

def compute_offset_curves(pts, offset):
    tang = np.gradient(pts, axis=0)
    norms = np.linalg.norm(tang,axis=1,keepdims=True); norms[norms==0]=1
    tang /= norms
    normals = np.column_stack([-tang[:,1], tang[:,0]])
    return pts - normals*offset, pts + normals*offset  # inner, outer

def segment_track(svg_path):
    tree, root = ET.parse(svg_path), None
    root = tree.getroot()
    ns = {'svg':root.tag.split('}')[0].strip('{')}
    raw=[]
    for e in root.findall('.//svg:path',ns):
        ctr = parse_path_d(e.get('d'))
        κ   = savgol_filter(compute_curvature(ctr), window_size, 3)
        raw += [(typ, ctr[idxs]) for typ,idxs in detect_segments(ctr,κ,curvature_threshold)]
        break

    if raw and raw[0][0]=='straight':
        raw = raw[1:]+raw[:1]

    recomb=[]
    for typ,pts in raw:
        if recomb and recomb[-1][0]==typ:
            recomb[-1] = (typ, np.vstack([recomb[-1][1], pts]))
        else:
            recomb.append((typ, pts))

    segs=[]; last_end=None
    for typ,pts in recomb:
        if last_end is not None and not np.allclose(pts[0], last_end):
            segs.append({'type':'straight','p0':last_end.tolist(),'p1':pts[0].tolist()})
        if typ=='straight':
            segs.append({'type':'straight','p0':pts[0].tolist(),'p1':pts[-1].tolist()})
            last_end = pts[-1]
        else:
            bez = fit_cubic_bezier(pts)
            if angle_between(bez[1]-bez[0], bez[-1]-bez[-2])<bezier_to_line_angle:
                segs.append({'type':'straight','p0':bez[0].tolist(),'p1':bez[-1].tolist()})
                last_end = bez[-1]
            else:
                segs.append({'type':'curve','controls':bez.tolist()})
                last_end = bez[-1]
    return segs

def load_and_plot(svg_path):
    segs = segment_track(svg_path)

    plt.figure()
    for seg in segs:
        if seg['type']=='straight':
            a,b = np.array(seg['p0']),np.array(seg['p1'])
            plt.plot([a[0],b[0]],[a[1],b[1]],'b-',lw=2)
            inn,out = compute_offset_curves(np.vstack([a,b]), offset)
            plt.plot(inn[:,0],inn[:,1],'b--',lw=1)
            plt.plot(out[:,0],out[:,1],'b--',lw=1)
        else:
            ctr = np.array(seg['controls'])
            t = np.linspace(0,1,200)
            curve = np.array([(1-ti)**3*ctr[0]
                              +3*(1-ti)**2*ti*ctr[1]
                              +3*(1-ti)*ti**2*ctr[2]
                              +ti**3*ctr[3] for ti in t])
            inn,out = compute_offset_curves(curve, offset)
            plt.plot(inn[:,0],inn[:,1],'r--',lw=1)
            plt.plot(out[:,0],out[:,1],'r--',lw=1)

    plt.axis('equal')
    plt.grid(True)

    # ——— Guardar SVG escalado (viewBox pequeño) ———
    out_dir = 'param_svg'
    os.makedirs(out_dir, exist_ok=True)

    # calcular bounding box
    all_xy = []
    for seg in segs:
        if seg['type']=='straight':
            all_xy += [seg['p0'], seg['p1']]
        else:
            all_xy += seg['controls']
    all_xy = np.vstack(all_xy)
    xmin,ymin = all_xy.min(axis=0)
    xmax,ymax = all_xy.max(axis=0)
    width, height = xmax-xmin, ymax-ymin

    svg_root = ET.Element('svg', xmlns="http://www.w3.org/2000/svg",
                          attrib={
                            'viewBox':f"{xmin:.3f} {ymin:.3f} {width:.3f} {height:.3f}",
                            'width':'400',   # ajustar tamaño deseado
                            'height':'400'
                          })

    def make_path(coords, color, dash=''):
        d = 'M ' + ' L '.join(f'{x:.3f},{y:.3f}' for x,y in coords)
        attrib={'d':d,'stroke':color,'fill':'none'}
        if dash: attrib['stroke-dasharray']=dash
        return ET.Element('path',attrib)

    for seg in segs:
        if seg['type']=='straight':
            a,b = seg['p0'],seg['p1']
            svg_root.append(make_path([a,b],'#0000ff'))
            inn,out = compute_offset_curves(np.vstack([a,b]), offset)
            svg_root.append(make_path(inn.tolist(),'#0000ff','5,5'))
            svg_root.append(make_path(out.tolist(),'#0000ff','5,5'))
        else:
            ctr = np.array(seg['controls'])
            t = np.linspace(0,1,200)
            curve = np.array([(1-ti)**3*ctr[0]
                              +3*(1-ti)**2*ti*ctr[1]
                              +3*(1-ti)*ti**2*ctr[2]
                              +ti**3*ctr[3] for ti in t])
            inn,out = compute_offset_curves(curve, offset)
            svg_root.append(make_path(inn.tolist(),'#ff0000','5,5'))
            svg_root.append(make_path(out.tolist(),'#ff0000','5,5'))

    track = os.path.splitext(os.path.basename(svg_path))[0]
    out_f = os.path.join(out_dir, f"{track}_param.svg")
    ET.ElementTree(svg_root).write(out_f)
    print("SVG guardado en:", out_f)

    plt.show()

if __name__=='__main__':
    root=Tk(); root.withdraw()
    fn=askopenfilename(title="Selecciona SVG",filetypes=[("SVG","*.svg")])
    if fn:
        load_and_plot(fn)
    else:
        print("No se seleccionó archivo.")

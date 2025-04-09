import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import re

# ——— Parámetros de robot y escala ———
robot_width_m = 0.30       # ancho del robot en metros
scale = 1.0                # unidades SVG por metro (ajusta si tu SVG usa otra unidad)
offset = (robot_width_m / 2) * scale

# ——— Parser SVG ———
COMMANDS = set('MmLlCcZz')
TOKEN_RE = re.compile(r'[MmLlCcZz]|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?')

def parse_path_d(d):
    tokens = TOKEN_RE.findall(d)
    i, cur, start = 0, np.zeros(2), np.zeros(2)
    points = []

    while i < len(tokens):
        cmd = tokens[i]

        if cmd in ('M','m'):
            is_rel = (cmd == 'm'); i += 1
            x, y = float(tokens[i]), float(tokens[i+1])
            cur = cur + [x,y] if is_rel else [x,y]
            start = cur.copy(); points.append(cur.copy()); i += 2
            # múltiples pares tras M
            while i+1 < len(tokens) and tokens[i] not in COMMANDS:
                x, y = float(tokens[i]), float(tokens[i+1])
                cur = cur + [x,y] if is_rel else [x,y]
                points.append(cur.copy()); i += 2

        elif cmd in ('L','l'):
            is_rel = (cmd == 'l'); i += 1
            while i+1 < len(tokens) and tokens[i] not in COMMANDS:
                x, y = float(tokens[i]), float(tokens[i+1])
                cur = cur + [x,y] if is_rel else [x,y]
                points.append(cur.copy()); i += 2

        elif cmd in ('C','c'):
            is_rel = (cmd == 'c'); i += 1
            while i+5 < len(tokens) and tokens[i] not in COMMANDS:
                coords = list(map(float, tokens[i:i+6]))
                if is_rel:
                    p1 = cur + coords[0:2]; p2 = cur + coords[2:4]; p3 = cur + coords[4:6]
                else:
                    p1, p2, p3 = np.array(coords[0:2]), np.array(coords[2:4]), np.array(coords[4:6])
                # muestreo de la curva
                for t in np.linspace(0, 1, 20):
                    pt = (1-t)**3 * cur \
                         + 3*(1-t)**2 * t * p1 \
                         + 3*(1-t) * t**2 * p2 \
                         + t**3 * p3
                    points.append(pt)
                cur = p3.copy(); i += 6

        elif cmd in ('Z','z'):
            points.append(start.copy()); cur = start.copy(); i += 1

        else:
            raise ValueError(f"Token inesperado: {cmd!r}")

    return np.vstack(points)

# ——— Cálculo de cotas paralelas ———
def compute_offset_curves(pts, offset):
    """
    Dado un array Nx2 de puntos (centerline), devuelve dos arrays Nx2:
    inner = offset negativo, outer = offset positivo, según normales.
    """
    # 1) calcular tangentes
    tangents = np.zeros_like(pts)
    tangents[1:-1] = (pts[2:] - pts[:-2]) / 2
    tangents[0]    = pts[1] - pts[0]
    tangents[-1]   = pts[-1] - pts[-2]
    # 2) normalizar
    norms = np.linalg.norm(tangents, axis=1, keepdims=True)
    norms[norms == 0] = 1
    tangents /= norms
    # 3) normales
    normals = np.column_stack([-tangents[:,1], tangents[:,0]])
    # 4) offset
    inner = pts - normals * offset
    outer = pts + normals * offset
    return inner, outer

# ——— Carga y plot ———
def load_and_plot(svg_path, path_id=None):
    tree = ET.parse(svg_path)
    root = tree.getroot()
    ns = {'svg': root.tag.split('}')[0].strip('{')}

    for elem in root.findall('.//svg:path', ns):
        if path_id is None or elem.get('id') == path_id:
            # 1) extraer centerline
            center_pts = parse_path_d(elem.get('d'))
            # 2) generar inner/outer
            inner_pts, outer_pts = compute_offset_curves(center_pts, offset)
            # 3) plot
            plt.figure()
            plt.plot(inner_pts[:,0], inner_pts[:,1], '--', label='Cota interna')
            plt.plot(center_pts[:,0], center_pts[:,1], '-',  label='Centro')
            plt.plot(outer_pts[:,0], outer_pts[:,1], '--', label='Cota externa')
            plt.axis('equal')
            plt.title("Pista con cotas interna y externa")
            plt.xlabel("x"); plt.ylabel("y")
            plt.legend(); plt.grid(True)
            plt.show()
            return

    raise RuntimeError("No encontré ningún <path> válido en el SVG.")

if __name__ == '__main__':
    svg_file = r"C:\Users\anyax\Desktop\SS CODIGO\Servicio-Social\Pistas con Direccion\Autódromo_Hermanos_Rodríguez_2015 (Bandera y Direccion).svg"
    load_and_plot(svg_file)

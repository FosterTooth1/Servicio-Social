import tkinter as tk
from tkinter import filedialog
from svgpathtools import svg2paths
import numpy as np

def seleccionar_svg():
    """Abre un diálogo para seleccionar un archivo SVG y devuelve su ruta."""
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(
        title="Selecciona un archivo SVG de circuito",
        filetypes=[("Archivos SVG", "*.svg")]
    )
    root.destroy()
    return file_path

def seleccionar_destino_csv(default_name="pista_puntos.csv"):
    """Abre un diálogo para elegir dónde y con qué nombre guardar el CSV."""
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.asksaveasfilename(
        title="Guardar puntos extraídos como",
        defaultextension=".csv",
        initialfile=default_name,
        filetypes=[("Archivos CSV", "*.csv")]
    )
    root.destroy()
    return file_path

def extraer_puntos_de_svg(svg_file, sample_points=None):
    """Extrae puntos de la trayectoria más larga dentro de un SVG."""
    paths, _ = svg2paths(svg_file)
    longest_path = max(paths, key=len)
    print(f"El path más largo tiene {len(longest_path)} segmentos")
    if sample_points is None:
        sample_points = np.linspace(0, 1, 100)
    return [(longest_path.point(t).real, longest_path.point(t).imag) for t in sample_points]

def guardar_a_csv(puntos, output_filename):
    """Guarda la lista de tuplas (x, y) en un archivo CSV."""
    with open(output_filename, "w", encoding="utf-8") as f:
        f.write("x,y\n")
        for x, y in puntos:
            f.write(f"{x},{y}\n")
    print(f"Puntos extraídos y guardados en {output_filename}")

if __name__ == "__main__":
    # 1) Selección de SVG
    svg_path = seleccionar_svg()
    if not svg_path:
        print("No se seleccionó ningún SVG. Saliendo.")
        exit()

    # 2) Definir los ts donde muestrear
    sample_ts = [0.01, 0.10, 0.50, 0.99]  # o usa None para 100 puntos uniformes
    puntos = extraer_puntos_de_svg(svg_path, sample_points=sample_ts)

    # 3) Selección de destino CSV
    csv_path = seleccionar_destino_csv()
    if not csv_path:
        print("No se definió ruta de salida para el CSV. Saliendo.")
        exit()

    # 4) Guardar CSV
    guardar_a_csv(puntos, csv_path)

import numpy as np
import pandas as pd
from tkinter import Tk
from tkinter.filedialog import askopenfilename, asksaveasfilename

def compute_track_boundaries(df: pd.DataFrame, closed: bool = True) -> pd.DataFrame:
    """
    A partir de un DataFrame con columnas [x_m, y_m, w_tr_right_m, w_tr_left_m],
    devuelve un DataFrame con:
      x_central, y_central, x_border1, y_border1, x_border2, y_border2
    donde border1 es el externo y border2 el interno.
    """
    x = df['x_m'].to_numpy()
    y = df['y_m'].to_numpy()
    d_ext = df['w_tr_right_m'].to_numpy()
    d_int = df['w_tr_left_m'].to_numpy()

    # Tangentes
    if closed:
        dx = np.roll(x, -1) - x
        dy = np.roll(y, -1) - y
    else:
        dx = np.empty_like(x)
        dy = np.empty_like(y)
        dx[:-1] = x[1:] - x[:-1]
        dy[:-1] = y[1:] - y[:-1]
        dx[-1] = dx[-2]
        dy[-1] = dy[-2]

    # Normales unitarias (90° hacia afuera)
    lengths = np.hypot(dx, dy)
    nx =  dy / lengths
    ny = -dx / lengths

    # Bordes exterior e interior
    x_ext = x + d_ext * nx
    y_ext = y + d_ext * ny
    x_int = x - d_int * nx
    y_int = y - d_int * ny

    return pd.DataFrame({
        'x_central':  x,
        'y_central':  y,
        'x_border1':  x_ext,
        'y_border1':  y_ext,
        'x_border2':  x_int,
        'y_border2':  y_int,
    })

def main():
    # Ocultamos la ventana principal de tkinter
    root = Tk()
    root.withdraw()

    # Selección dinámica de archivo de entrada
    input_path = askopenfilename(
        title="Selecciona el CSV de entrada",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )
    if not input_path:
        print("No se seleccionó archivo de entrada. Saliendo.")
        return

    # Detectar si la primera línea es la cabecera comentada
    with open(input_path, 'r') as f:
        first_line = f.readline().strip()

    if first_line.startswith('#'):
        # Cabecera comentada: la quitamos y usamos su contenido como nombres
        col_names = [c.strip() for c in first_line.lstrip('#').split(',')]
        df_in = pd.read_csv(
            input_path,
            comment=None,
            header=None,
            skiprows=1,
            names=col_names
        )
    else:
        # Cabecera normal
        df_in = pd.read_csv(input_path)

    # Selección dinámica de destino de salida
    output_path = asksaveasfilename(
        title="Guardar CSV de salida como",
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )
    if not output_path:
        print("No se seleccionó ruta de salida. Saliendo.")
        return

    # Procesar y guardar
    df_out = compute_track_boundaries(df_in, closed=True)
    df_out.to_csv(output_path, index=False, float_format='%.18e')

    print(f"Archivo de salida guardado en:\n  {output_path}")


if __name__ == '__main__':
    main()

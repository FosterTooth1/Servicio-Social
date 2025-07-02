import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename

def seleccionar_csv():
    root = Tk()
    root.withdraw()  # Oculta la ventana principal de Tkinter
    filename = askopenfilename(
        title="Selecciona el archivo CSV",
        filetypes=[("Archivos CSV", "*.csv"), ("Todos los archivos", "*.*")]
    )
    root.destroy()
    if not filename:
        raise FileNotFoundError("No se ha seleccionado ningún archivo CSV.")
    return filename

def visualizar_csv(filename):
    # Leer CSV
    df = pd.read_csv(filename)
    # Verificar columnas esperadas
    expected = ["x_central", "y_central", "x_border1", "y_border1", "x_border2", "y_border2"]
    for col in expected:
        if col not in df.columns:
            raise ValueError(f"Columna esperada no encontrada en el CSV: {col}")

    # Extraer series
    x_c = df["x_central"].values
    y_c = df["y_central"].values
    x_b1 = df["x_border1"].values
    y_b1 = df["y_border1"].values
    x_b2 = df["x_border2"].values
    y_b2 = df["y_border2"].values

    # Crear figura
    plt.figure(figsize=(8, 6))
    plt.scatter(x_c, y_c, label="Central", s=20)
    plt.scatter(x_b1, y_b1, label="Borde Externo", s=20)
    plt.scatter(x_b2, y_b2, label="Borde Interno", s=20)

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Visualización de puntos: central, externo e interno")
    plt.legend()
    plt.gca().set_aspect('equal', 'box')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def main():
    try:
        csv_file = seleccionar_csv()
        print(f"Archivo seleccionado: {csv_file}")
        visualizar_csv(csv_file)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

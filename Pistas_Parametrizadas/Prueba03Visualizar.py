import os
import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename

def visualizar_csv(filename):
    # Leer CSV
    df = pd.read_csv(filename)
    # Columnas que esperamos
    expected = [
        "x_central", "y_central",
        "x_border1", "y_border1",
        "x_border2", "y_border2",
        "curva"
    ]
    for col in expected:
        if col not in df.columns:
            raise ValueError(f"Columna esperada no encontrada en el CSV: {col}")

    # Extraer arrays
    x_c = df["x_central"].values
    y_c = df["y_central"].values
    x_b1 = df["x_border1"].values
    y_b1 = df["y_border1"].values
    x_b2 = df["x_border2"].values
    y_b2 = df["y_border2"].values
    is_curve = df["curva"].values.astype(bool)  # True si curva, False si recta

    # Máscaras
    m_curve = is_curve
    m_straight = ~is_curve

    # Crear figura
    plt.figure(figsize=(8, 6))

    # Línea central: puntos de recta en azul, de curva en rojo
    plt.scatter(
        x_c[m_straight], y_c[m_straight],
        color='blue', marker='o', s=20, label='Central (recta)'
    )
    plt.scatter(
        x_c[m_curve], y_c[m_curve],
        color='red', marker='o', s=20, label='Central (curva)'
    )

    # Bordes: mismos colores pero con otro marcador
    plt.scatter(
        x_b1[m_straight], y_b1[m_straight],
        color='blue', marker='x', s=20, label='Borde ext. (recta)'
    )
    plt.scatter(
        x_b1[m_curve], y_b1[m_curve],
        color='red', marker='x', s=20, label='Borde ext. (curva)'
    )
    plt.scatter(
        x_b2[m_straight], y_b2[m_straight],
        color='blue', marker='^', s=20, label='Borde int. (recta)'
    )
    plt.scatter(
        x_b2[m_curve], y_b2[m_curve],
        color='red', marker='^', s=20, label='Borde int. (curva)'
    )

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Visualización de la pista: rectas vs curvas")
    plt.legend(loc='best', fontsize='small')
    plt.gca().set_aspect('equal', 'box')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def main():
    # Abrir diálogo para seleccionar CSV
    root = Tk()
    root.withdraw()
    csv_file = askopenfilename(
        title="Selecciona un archivo CSV",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )
    if not csv_file:
        print("No se seleccionó ningún archivo. Saliendo.")
        return

    try:
        visualizar_csv(csv_file)
    except Exception as e:
        print(f"Error al procesar el archivo: {e}")

if __name__ == "__main__":
    main()

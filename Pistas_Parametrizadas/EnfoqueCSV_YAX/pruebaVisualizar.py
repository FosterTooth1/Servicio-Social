import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from tkinter import Tk
from tkinter.filedialog import askopenfilename
from matplotlib.lines import Line2D

def visualizar_csv(filename):
    # Leer CSV
    df = pd.read_csv(filename)
    
    # Verificar columnas requeridas incluyendo 'is_curve'
    required_columns = [
        "x_central", "y_central",
        "x_border1", "y_border1",
        "x_border2", "y_border2",
        "is_curve"
    ]
    
    for col in required_columns:
        if col not in df.columns:
            raise ValueError(f"Columna requerida no encontrada: {col}")

    # Extraer coordenadas y datos de curvatura
    x_c = df["x_central"].values
    y_c = df["y_central"].values
    x_b1 = df["x_border1"].values
    y_b1 = df["y_border1"].values
    x_b2 = df["x_border2"].values
    y_b2 = df["y_border2"].values
    is_curve = df["is_curve"].values

    # Crear máscaras para rectas y curvas
    straight_mask = (is_curve == 0)
    curve_mask = (is_curve == 1)
    
    # Crear figura con tamaño adecuado
    plt.figure(figsize=(14, 10))
    
    # 1. Dibujar línea central con colores según curvatura
    # Puntos de rectas (azul)
    plt.scatter(
        x_c[straight_mask], y_c[straight_mask], 
        color='blue', s=40, 
        edgecolors='black', linewidths=0.7,
        label='Centro (Recta)'
    )
    
    # Puntos de curvas (rojo)
    plt.scatter(
        x_c[curve_mask], y_c[curve_mask], 
        color='red', s=40, 
        edgecolors='black', linewidths=0.7,
        label='Centro (Curva)'
    )
    
    # 2. Dibujar puntos de los bordes con colores según curvatura
    # Borde interno (cruces)
    plt.scatter(
        x_b1[straight_mask], y_b1[straight_mask], 
        color='blue', marker='x', 
        s=30, linewidths=1.5,
        label='Borde interno (Recta)'
    )
    plt.scatter(
        x_b1[curve_mask], y_b1[curve_mask], 
        color='red', marker='x', 
        s=30, linewidths=1.5,
        label='Borde interno (Curva)'
    )
    
    # Borde externo (triángulos)
    plt.scatter(
        x_b2[straight_mask], y_b2[straight_mask], 
        color='blue', marker='^', 
        s=30, edgecolors='black', linewidths=0.5,
        label='Borde externo (Recta)'
    )
    plt.scatter(
        x_b2[curve_mask], y_b2[curve_mask], 
        color='red', marker='^', 
        s=30, edgecolors='black', linewidths=0.5,
        label='Borde externo (Curva)'
    )
    
    # 3. Dibujar líneas de conexión con colores según curvatura
    # Encontrar segmentos continuos
    curve_segments = []
    start_idx = 0
    for i in range(1, len(is_curve)):
        if is_curve[i] != is_curve[i-1] or i == len(is_curve)-1:
            end_idx = i
            curve_segments.append((start_idx, end_idx, is_curve[i-1]))
            start_idx = i

    # Dibujar líneas para cada segmento
    for start, end, curve_type in curve_segments:
        color = 'red' if curve_type == 1 else 'blue'
        plt.plot(x_c[start:end], y_c[start:end], 
                 color=color, linewidth=0.8, alpha=0.5)
        plt.plot(x_b1[start:end], y_b1[start:end], 
                 color=color, linewidth=0.8, alpha=0.5)
        plt.plot(x_b2[start:end], y_b2[start:end], 
                 color=color, linewidth=0.8, alpha=0.5)

    # 4. Añadir flechas de dirección en puntos estratégicos
    arrow_step = len(x_c) // 8  # 8 flechas a lo largo del circuito
    for i in range(0, len(x_c), arrow_step):
        if i < len(x_c) - 1:
            dx = x_c[i+1] - x_c[i]
            dy = y_c[i+1] - y_c[i]
            plt.arrow(
                x_c[i], y_c[i], 
                dx*0.8, dy*0.8, 
                head_width=2.5, 
                head_length=3.5, 
                fc='black', 
                ec='black',
                alpha=0.7
            )

    # 5. Configuración avanzada del gráfico
    plt.title("Visualización de Circuito F1: Rectas vs Curvas", fontsize=16, fontweight='bold')
    plt.xlabel("Coordenada X", fontsize=12)
    plt.ylabel("Coordenada Y", fontsize=12)
    
    # Crear leyenda personalizada
    legend_elements = [
        Line2D([0], [0], color='blue', lw=2, label='Rectas'),
        Line2D([0], [0], color='red', lw=2, label='Curvas'),
        Line2D([0], [0], marker='o', color='blue', markerfacecolor='blue', 
               markersize=10, label='Centro (Recta)', markeredgecolor='black'),
        Line2D([0], [0], marker='o', color='red', markerfacecolor='red', 
               markersize=10, label='Centro (Curva)', markeredgecolor='black'),
        Line2D([0], [0], marker='x', color='blue', lw=0, 
               markersize=10, label='Borde interno (Recta)'),
        Line2D([0], [0], marker='x', color='red', lw=0, 
               markersize=10, label='Borde interno (Curva)'),
        Line2D([0], [0], marker='^', color='blue', lw=0, 
               markersize=10, label='Borde externo (Recta)'),
        Line2D([0], [0], marker='^', color='red', lw=0, 
               markersize=10, label='Borde externo (Curva)')
    ]
    
    plt.legend(handles=legend_elements, loc='best', fontsize=9)

    plt.grid(True, linestyle='--', alpha=0.3)
    
    # Mantener relación de aspecto
    plt.gca().set_aspect('equal', 'box')
    
    # Ajustar límites con margen
    all_x = np.concatenate([x_c, x_b1, x_b2])
    all_y = np.concatenate([y_c, y_b1, y_b2])
    
    x_margin = (np.max(all_x) - np.min(all_x)) * 0.1
    y_margin = (np.max(all_y) - np.min(all_y)) * 0.1
    
    plt.xlim(np.min(all_x) - x_margin, np.max(all_x) + x_margin)
    plt.ylim(np.min(all_y) - y_margin, np.max(all_y) + y_margin)
    
    plt.tight_layout()
    plt.show()

def main():
    # Configurar ventana de selección de archivos
    root = Tk()
    root.withdraw()  # Ocultar ventana principal
    
    # Solicitar archivo CSV
    csv_file = askopenfilename(
        title="Selecciona el archivo CSV del circuito",
        filetypes=[("Archivos CSV", "*.csv"), ("Todos los archivos", "*.*")]
    )
    
    if not csv_file:
        print("No se seleccionó ningún archivo")
        return
    
    try:
        visualizar_csv(csv_file)
    except Exception as e:
        print(f"Error al procesar el archivo: {str(e)}")

if __name__ == "__main__":
    main()
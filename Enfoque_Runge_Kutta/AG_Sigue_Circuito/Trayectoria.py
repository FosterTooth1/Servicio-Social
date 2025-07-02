import csv
import matplotlib.pyplot as plt

# Listas para almacenar los datos
tiempos = []
x_vals = []
y_vals = []

# Leer el archivo CSV
with open('trayectoria.csv', 'r') as csvfile:
    lector = csv.DictReader(csvfile)
    for fila in lector:
        tiempos.append(float(fila['time']))
        x_vals.append(float(fila['x']))
        y_vals.append(float(fila['y']))

# Definir todos los puntos objetivo (coinciden con el código C)
objetivos = [
    (2.0, 3.0),
    (5.0, 7.0),
    (8.0, 10.0)
]

# Graficar la trayectoria (x vs y)
plt.figure(figsize=(8, 6))
plt.plot(x_vals, y_vals, marker='o', linestyle='-', label='Trayectoria')

# Plotear TODOS los puntos objetivo
for i, (x_obj, y_obj) in enumerate(objetivos):
    plt.scatter(x_obj, y_obj, color='red', marker='*', s=150, label=f'Objetivo P{i}' if i == 0 else "")

plt.xlabel('Posición X')
plt.ylabel('Posición Y')
plt.title('Trayectoria del Robot con Múltiples Objetivos')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
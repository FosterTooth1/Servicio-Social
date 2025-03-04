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

# Graficar la trayectoria (x vs y)
plt.figure(figsize=(8, 6))
plt.plot(x_vals, y_vals, marker='o', linestyle='-', label='Trayectoria')

# Plotear el punto objetivo (15, 0)
plt.scatter(15, 7.5, color='red', marker='*', s=150, label='Objetivo (15, 0)')

plt.xlabel('Posición X')
plt.ylabel('Posición Y')
plt.title('Trayectoria del Robot')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()

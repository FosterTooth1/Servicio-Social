from svgpathtools import svg2paths
import numpy as np

# Ruta del archivo SVG
svg_file = "./Pistas Limpias/Autódromo_Hermanos_Rodríguez_2015 (Limpia).svg"
"""
Albert_Park_Circuit_2021 (Limpia).svg
Austin_circuit (Limpia).svg
Autódromo_Hermanos_Rodríguez_2015 (Limpia).svg
Autódromo_José_Carlos_Pace_(AKA_Interlagos)_track_map (Limpia).svg
Autódromo_Oscar_y_Juan_Gálvez_Circuito_N°_15 (Limpia).svg
Circuit_de_Catalunya_moto_2021 (Limpia).svg
Circuit_Nürburgring-1927 (Limpia).svg
Circuit_Paul_Ricard_2020_layout_map (Limpia).svg
Circuit_Silverstone_2011 (Limpia).svg
Hungaroring (Limpia).svg
Istanbul_park (Limpia).svg
Spa-Francorchamps_of_Belgium (Limpia).svg
Suzuka_circuit_map--2005 (Limpia).svg
"""

# Cargar paths desde el archivo SVG
paths, attributes = svg2paths(svg_file)

# Encontrar el path más largo (con más segmentos)
longest_path = max(paths, key=len)
trayectoria = longest_path
print(f"El path más largo tiene {len(longest_path)} segmentos")

# Número de puntos a muestrear en el path más largo
num_samples = 100  # Ajusta según la precisión deseada

# Lista para almacenar los puntos extraídos
all_points = []

"""
for t in np.linspace(0, 1, num_samples):  # Muestreo uniforme en la trayectoria
    point = longest_path.point(t)
    all_points.append((point.real, point.imag))
    
"""
# Tiempo es el largo de la pista 1 / largo de la pista real   ->> 
point = trayectoria.point(0.01)
all_points.append((point.real, point.imag))
point = trayectoria.point(0.10)
all_points.append((point.real, point.imag))
point = trayectoria.point(0.50)
all_points.append((point.real, point.imag))
point = trayectoria.point(0.99)
all_points.append((point.real, point.imag))

# Guardar puntos en un archivo CSV
with open("pista_puntos.csv", "w") as f:
    f.write("x,y\n")
    for x, y in all_points:
        f.write(f"{x},{y}\n")

print("Puntos extraídos y guardados en pista_puntos.csv")

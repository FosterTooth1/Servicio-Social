#!/usr/bin/env python3
"""
Script para convertir un CSV de waypoints en la especificación XML de TORCS (.trk)
Formato CSV esperado:
    x_central,y_central,x_border1,y_border1,x_border2,y_border2
El script genera un archivo XML con:
  - sección Header mínima
  - sección Local Info con valores por defecto
  - sección "Piste" donde cada nodo incluye la posición central (X, Y, Z=0)
    y el ancho de pista calculado como la distancia media a los bordes.

Uso:
    python csv_to_trk.py track.csv output.trk
"""
import sys
import csv
import math
import xml.etree.ElementTree as ET

if len(sys.argv) != 3:
    print("Uso: python csv_to_trk.py input.csv output.trk")
    sys.exit(1)

input_csv = sys.argv[1]
output_trk = sys.argv[2]

# Leer CSV
waypoints = []
with open(input_csv, 'r', newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        x_c = float(row['x_central'])
        y_c = float(row['y_central'])
        x_b1 = float(row['x_border1'])
        y_b1 = float(row['y_border1'])
        x_b2 = float(row['x_border2'])
        y_b2 = float(row['y_border2'])
        # Calcular ancho aproximado (distancia media a los dos bordes)
        d1 = math.hypot(x_b1 - x_c, y_b1 - y_c)
        d2 = math.hypot(x_b2 - x_c, y_b2 - y_c)
        width = (d1 + d2)
        waypoints.append((x_c, y_c, width))

# Crear estructura XML
root = ET.Element('track')

# Header (editar campos a conveniencia)
header = ET.SubElement(root, 'section', {'name': 'Header'})
ET.SubElement(header, 'attstr', {'name': 'name', 'val': 'custom-track'})
ET.SubElement(header, 'attstr', {'name': 'category', 'val': 'circuit'})
ET.SubElement(header, 'attnum', {'name': 'version', 'val': '1'})

# Local Info (valores por defecto)
local = ET.SubElement(root, 'section', {'name': 'Local Info'})
ET.SubElement(local, 'attstr', {'name': 'station', 'val': 'NONE'})
ET.SubElement(local, 'attnum', {'name': 'time of day', 'unit': 'hour', 'val': '12.0'})
ET.SubElement(local, 'attnum', {'name': 'sun ascension', 'unit': 'deg', 'val': '45'})

# Sección de nodos (pista)
piste = ET.SubElement(root, 'Piste')
for idx, (x, y, w) in enumerate(waypoints):
    # Cada nodo puede llamarse Corner o Node según la versión de TORCS
    node = ET.SubElement(piste, 'Node', {
        'X': f"{x:.6f}",
        'Y': f"{y:.6f}",
        'Z': "0.000000",
        'width': f"{w:.6f}"  # ancho total de pista en ese punto
    })

# Serializar y guardar
tree = ET.ElementTree(root)
ET.indent(tree, space="  ", level=0)  # Pythón 3.9+
tree.write(output_trk, encoding='utf-8', xml_declaration=True)
print(f"Generado {output_trk} con {len(waypoints)} nodos.")

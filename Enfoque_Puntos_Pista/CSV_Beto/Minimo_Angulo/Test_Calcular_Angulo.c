#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<time.h>
#include<stdbool.h>
#include<math.h>

#define M_PI 3.14159265358979323846

// Nueva estructura de punto con coordenadas centrales y dos bordes
typedef struct {
    double x;              // Posición actual (optimizada)
    double y;
} Punto;

// Estructura del circuito
typedef struct {
    Punto *puntos;
} Circuito;

// Estructura de la solución
typedef struct {
    Circuito ruta;      // Ruta actual
    double fitness;     // Fitness actual
} Solucion;

// Crear puntos para test
Punto crear_punto(double x, double y) {
    Punto p;
    p.x = x;
    p.y = y;
    return p;
}

double calcular_angulo_tres_puntos(const Punto *p1, const Punto *p2, const Punto *p3) {
    double vec1_x = p2->x - p1->x;
    double vec1_y = p2->y - p1->y;
    double vec2_x = p3->x - p2->x;
    double vec2_y = p3->y - p2->y;

    double dot = vec1_x * vec2_x + vec1_y * vec2_y;
    double mag1 = sqrt(vec1_x * vec1_x + vec1_y * vec1_y);
    double mag2 = sqrt(vec2_x * vec2_x + vec2_y * vec2_y);
    
    if (mag1 == 0.0 || mag2 == 0.0) {
        return 0.0;
    }
    
    double cos_theta = dot / (mag1 * mag2);
    // Asegurar valor dentro de rango [-1, 1]
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    
    double ang_rad = acos(cos_theta);
    return ang_rad * (180.0 / M_PI);
}

int main() {
    // Crear puntos de prueba
    Punto p1 = crear_punto(0.0, 0.0);
    Punto p2 = crear_punto(1.0, -5.0);
    Punto p3 = crear_punto(2.0, 0.0);

    // Calcular ángulo entre los puntos
    double angulo = calcular_angulo_tres_puntos(&p1, &p2, &p3);
    
    // Imprimir el resultado
    printf("Angulo entre los puntos: %.2f grados\n", angulo);
    
    return 0;
}
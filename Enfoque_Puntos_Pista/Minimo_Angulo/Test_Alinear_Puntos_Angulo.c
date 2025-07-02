#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

typedef struct {
    double x;
    double y;
    double limite_x_externo;
    double limite_y_externo;
    double limite_x_interno;
    double limite_y_interno;
} Punto;    

typedef struct {
    Punto *puntos;
    int num_puntos;
} Circuito;

void leer_puntos(Circuito *Circuito);
void imprimir_puntos(Circuito *Circuito);
void liberar_puntos(Circuito *Circuito);
double calcular_angulo_tres_puntos(Punto p1, Punto p2, Punto p3);
void rectificar_circuito(Circuito *Circuito);
double evaluar_fitness(Circuito *Circuito);

void leer_puntos(Circuito *Circuito) {
    FILE *archivo = fopen("pista_escalada.csv", "r");
    if (archivo == NULL) {
        perror("Error al abrir el archivo");
        exit(EXIT_FAILURE);
    }

    char encabezado[256];
    if (fgets(encabezado, sizeof(encabezado), archivo) == NULL) {
        fclose(archivo);
        fprintf(stderr, "Error: Archivo vacío o sin encabezado\n");
        exit(EXIT_FAILURE);
    }

    Circuito->num_puntos = 0;
    Circuito->puntos = NULL;

    char linea[256];
    while (fgets(linea, sizeof(linea), archivo)) {
        Punto *temp = realloc(Circuito->puntos, (Circuito->num_puntos + 1) * sizeof(Punto));
        if (temp == NULL) {
            perror("Error al asignar memoria");
            fclose(archivo);
            free(Circuito->puntos);
            exit(EXIT_FAILURE);
        }
        Circuito->puntos = temp;
        
        int campos_leidos = sscanf(
            linea, 
            "%lf,%lf,%lf,%lf,%lf,%lf", 
            &Circuito->puntos[Circuito->num_puntos].x, 
            &Circuito->puntos[Circuito->num_puntos].y,
            &Circuito->puntos[Circuito->num_puntos].limite_x_externo,
            &Circuito->puntos[Circuito->num_puntos].limite_y_externo,
            &Circuito->puntos[Circuito->num_puntos].limite_x_interno,
            &Circuito->puntos[Circuito->num_puntos].limite_y_interno
        );
        
        if (campos_leidos != 6) {
            fprintf(stderr, "Error en el formato de la línea: %s", linea);
            fclose(archivo);
            free(Circuito->puntos);
            exit(EXIT_FAILURE);
        }
        
        Circuito->num_puntos++;
    }

    fclose(archivo);
}

void imprimir_puntos(Circuito *Circuito) {
    for (int i = 0; i < Circuito->num_puntos; i++) {
        printf("Punto %d: (%.2f, %.2f)\n", i + 1, Circuito->puntos[i].x, Circuito->puntos[i].y);
        printf("Limites Externos: (%.2f, %.2f)\n", Circuito->puntos[i].limite_x_externo, Circuito->puntos[i].limite_y_externo);
        printf("Limites Internos: (%.2f, %.2f)\n", Circuito->puntos[i].limite_x_interno, Circuito->puntos[i].limite_y_interno);
    }
}

double calcular_angulo_tres_puntos(Punto p1, Punto p2, Punto p3) {
    double vec1_x = p2.x - p1.x;
    double vec1_y = p2.y - p1.y;
    double vec2_x = p3.x - p2.x;
    double vec2_y = p3.y - p2.y;
    
    double dot = vec1_x * vec2_x + vec1_y * vec2_y;
    double mag1 = sqrt(vec1_x * vec1_x + vec1_y * vec1_y);
    double mag2 = sqrt(vec2_x * vec2_x + vec2_y * vec2_y);
    
    if (mag1 == 0 || mag2 == 0) return 0.0;
    
    double cos_theta = dot / (mag1 * mag2);
    cos_theta = fmax(fmin(cos_theta, 1.0), -1.0);
    
    return acos(cos_theta) * (180.0 / 3.141592);
}

void rectificar_circuito(Circuito *Circuito) {
    if (Circuito->num_puntos < 3) return;

    const double Pm = 0.1;
    const int Nm = 100;

    for (int i = 0; i < Circuito->num_puntos - 2; ++i) {
        Punto *p1 = &Circuito->puntos[i];
        Punto *p2 = &Circuito->puntos[i + 1];
        Punto *p3 = &Circuito->puntos[i + 2];
        
        double angulo = calcular_angulo_tres_puntos(*p1, *p2, *p3);
        printf("Angulo en trio %d-%d-%d: %.2f grados\n", i + 1, i + 2, i + 3, angulo);

        if (angulo > 0.1) {
            if ((double)rand() / RAND_MAX <= Pm) {
                double upper_x = fmax(p2->limite_x_externo, p2->limite_x_interno);
                double lower_x = fmin(p2->limite_x_externo, p2->limite_x_interno);
                double delta_x = fmin(upper_x - p2->x, p2->x - lower_x) / (upper_x - lower_x);
                double r = (double)rand() / RAND_MAX;
                double deltaq_x;

                if (r <= 0.5) {
                    deltaq_x = pow(2 * r + (1 - 2 * r) * pow(1 - delta_x, Nm + 1), 1.0 / (Nm + 1)) - 1;
                } else {
                    deltaq_x = 1 - pow(2 * (1 - r) + 2 * (r - 0.5) * pow(1 - delta_x, Nm + 1), 1.0 / (Nm + 1));
                }

                p2->x += deltaq_x * (upper_x - lower_x);
                p2->x = fmax(lower_x, fmin(p2->x, upper_x));
            }

            if ((double)rand() / RAND_MAX <= Pm) {
                double upper_y = fmax(p2->limite_y_externo, p2->limite_y_interno);
                double lower_y = fmin(p2->limite_y_externo, p2->limite_y_interno);
                double delta_y = fmin(upper_y - p2->y, p2->y - lower_y) / (upper_y - lower_y);
                double r = (double)rand() / RAND_MAX;
                double deltaq_y;

                if (r <= 0.5) {
                    deltaq_y = pow(2 * r + (1 - 2 * r) * pow(1 - delta_y, Nm + 1), 1.0 / (Nm + 1)) - 1;
                } else {
                    deltaq_y = 1 - pow(2 * (1 - r) + 2 * (r - 0.5) * pow(1 - delta_y, Nm + 1), 1.0 / (Nm + 1));
                }

                p2->y += deltaq_y * (upper_y - lower_y);
                p2->y = fmax(lower_y, fmin(p2->y, upper_y));
            }
        }
    }
}

double evaluar_fitness(Circuito *Circuito) {
    if (Circuito->num_puntos < 3) return 0.0;

    double suma_angulos = 0.0;
    for (int i = 0; i < Circuito->num_puntos - 2; ++i) {
        Punto p1 = Circuito->puntos[i];
        Punto p2 = Circuito->puntos[i + 1];
        Punto p3 = Circuito->puntos[i + 2];
        
        double angulo = calcular_angulo_tres_puntos(p1, p2, p3);
        suma_angulos += angulo;
    }
    return suma_angulos;
}

int main() {
    srand(time(NULL));
    Circuito circuito;
    leer_puntos(&circuito);
    
    printf("Fitness antes de rectificar: %.2f grados\n", evaluar_fitness(&circuito));
    rectificar_circuito(&circuito);
    printf("Fitness después de rectificar: %.2f grados\n", evaluar_fitness(&circuito));
    
    //imprimir_puntos(&circuito);
    liberar_puntos(&circuito);
    return 0;
}

void liberar_puntos(Circuito *Circuito) {
    free(Circuito->puntos);
    Circuito->puntos = NULL;
    Circuito->num_puntos = 0;
}
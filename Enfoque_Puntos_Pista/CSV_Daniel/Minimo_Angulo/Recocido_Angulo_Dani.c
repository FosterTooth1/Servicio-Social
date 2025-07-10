#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>

#define M_PI 3.14159265358979323846

// Nueva estructura de punto con coordenadas centrales y dos bordes
typedef struct {
    double x;              // Posición actual (optimizada)
    double y;
    double x_central;      // Centro de la pista
    double y_central;
    double x_border1;      // Borde 1
    double y_border1;
    double x_border2;      // Borde 2
    double y_border2;
} Punto;    

// Estructura del circuito
typedef struct {
    Punto *puntos;
} Circuito;

// Estructura de la solución
typedef struct
{
    Circuito ruta;      // Ruta actual
    double fitness;     // Fitness actual
} Solucion;

// Prototipos de funciones
void leer_puntos(Circuito *circuito, char* nombre_archivo, int *num_puntos);
void imprimir_puntos(Circuito *circuito, int num_puntos);
void liberar_puntos(Circuito *circuito);
void determinar_limites_interno_externo(const Punto *p, double *xi, double *yi, double *xe, double *ye);
double calcular_radio_tres_puntos(const Punto *p1, const Punto *p2, const Punto *p3);
void crear_individuo(Circuito *circuito, int num_puntos);
void rectificar_circuito(Circuito *circuito, int num_puntos);
double evaluar_fitness(Circuito *circuito, int num_puntos);
double probabilidad_aceptacion(double fitness_actual, double fitness_vecino, double temperatura);
void liberar_solucion(Solucion *actual);
void guardar_solucion_csv(const char *nombre_archivo, Solucion *sol, int num_puntos);

// Función auxiliar: double uniforme en [0,1)
static double rand_double_0_1() {
    return (double)rand() / ((double)RAND_MAX + 1.0);
}

// Determina qué borde es interno y cuál externo
void determinar_limites_interno_externo(const Punto *p, double *xi, double *yi, double *xe, double *ye) {
    *xi = p->x_border2;  // Borde interno (fijo)
    *yi = p->y_border2;
    *xe = p->x_border1;  // Borde externo (fijo)
    *ye = p->y_border1;
}

// Crea individuo inicial interpolando entre bordes interno y externo
void crear_individuo(Circuito *circuito, int num_puntos) {
    if (circuito == NULL || circuito->puntos == NULL) {
        fprintf(stderr, "crear_individuo: circuito o puntos NULL\n");
        return;
    }

    for (int i = 0; i < num_puntos; i++) {
        Punto *p = &circuito->puntos[i];
        double xi, yi, xe, ye;
        
        determinar_limites_interno_externo(p, &xi, &yi, &xe, &ye);

        // Vector entre bordes
        double dx = xe - xi;
        double dy = ye - yi;
        double dist2 = dx*dx + dy*dy;
        
        if (dist2 < 1e-12) {
            p->x = xi;
            p->y = yi;
            continue;
        }

        // Muestrear t en [0,1) para interpolar
        double t = rand_double_0_1();
        p->x = xi + t * dx;
        p->y = yi + t * dy;
    }
}

// Calcula el radio de la circunferencia que pasa por tres puntos
double calcular_radio_tres_puntos(const Punto *p1, const Punto *p2, const Punto *p3) {
    // Longitudes de los lados del triángulo:
    // a = distancia P2–P3,  b = distancia P1–P3,  c = distancia P1–P2
    double a = hypot(p2->x - p3->x, p2->y - p3->y);
    double b = hypot(p1->x - p3->x, p1->y - p3->y);
    double c = hypot(p1->x - p2->x, p1->y - p2->y);

    // Área del triángulo usando la fórmula de determinante:
    // A = ½ | x1(y2–y3) + x2(y3–y1) + x3(y1–y2) |
    double A = 0.5 * fabs(
        p1->x * (p2->y - p3->y) +
        p2->x * (p3->y - p1->y) +
        p3->x * (p1->y - p2->y)
    );

    // Si el área es cero, los puntos son colineales → radio despreciable
    if (A == 0.0) {
        return 0.0;
    }

    // Radio de la circunferencia: r = (a·b·c) / (4·A)
    return (a * b * c) / (4.0 * A);
}

// Rectifica el circuito usando parámetro t
void rectificar_circuito(Circuito *circuito, int num_puntos) {
    if (circuito == NULL || circuito->puntos == NULL) return;
    if (num_puntos < 3) return;

    const double Pm = 0.01;  // Probabilidad de modificación
    const int Nm = 100;     // Parámetro de distribución

    for (int i = 0; i < num_puntos - 2; ++i) {
        Punto *p1 = &circuito->puntos[i];
        Punto *p2 = &circuito->puntos[i + 1];
        Punto *p3 = &circuito->puntos[i + 2];
        
        // Determinar límites interno/externo para p2
        double xi, yi, xe, ye;
        determinar_limites_interno_externo(p2, &xi, &yi, &xe, &ye);

        // Vector entre bordes
        double vx = xe - xi;
        double vy = ye - yi;
        double len2 = vx*vx + vy*vy;
        if (len2 < 1e-12) {
            continue;
        }

        // Calcular t actual
        double t_actual;
        double wx = p2->x - xi;
        double wy = p2->y - yi;
        t_actual = (wx * vx + wy * vy) / len2;
        if (t_actual < 0.0) t_actual = 0.0;
        if (t_actual > 1.0) t_actual = 1.0;

        // Ángulo original
        double angulo_original = calcular_radio_tres_puntos(p1, p2, p3);
        double x_orig = p2->x;
        double y_orig = p2->y;

        // Intentar modificación con probabilidad Pm
        if (rand_double_0_1() <= Pm) {
            double delta_pos = fmin(t_actual, 1.0 - t_actual);
            double r = rand_double_0_1();
            double deltaq_t;

            if (r <= 0.5) {
                deltaq_t = pow(2 * r + (1 - 2 * r) * pow(1 - delta_pos, Nm + 1), 1.0 / (Nm + 1)) - 1;
            } else {
                deltaq_t = 1 - pow(2 * (1 - r) + 2 * (r - 0.5) * pow(1 - delta_pos, Nm + 1), 1.0 / (Nm + 1));
            }

            // Nuevo valor de t
            double t_nuevo = t_actual + deltaq_t;
            if (t_nuevo < 0.0) t_nuevo = 0.0;
            if (t_nuevo > 1.0) t_nuevo = 1.0;

            // Actualizar posición
            p2->x = xi + t_nuevo * vx;
            p2->y = yi + t_nuevo * vy;

            // Verificar si mejora el ángulo
            double nuevo_angulo = calcular_radio_tres_puntos(p1, p2, p3);
            if (nuevo_angulo > angulo_original) {
                // Revertir si empeora
                p2->x = x_orig;
                p2->y = y_orig;
            }
        }
    }
}

// Evalúa fitness como suma de ángulos
double evaluar_fitness(Circuito *circuito, int num_puntos) {
    if (circuito == NULL || circuito->puntos == NULL) return 0.0;
    if (num_puntos < 3) return 0.0;
    
    double suma_angulos = 0.0;
    for (int i = 0; i < num_puntos - 2; ++i) {
        const Punto *p1 = &circuito->puntos[i];
        const Punto *p2 = &circuito->puntos[i + 1];
        const Punto *p3 = &circuito->puntos[i + 2];
        
        double ang = calcular_radio_tres_puntos(p1, p2, p3);
        suma_angulos += ang;
    }
    return suma_angulos;
}

double probabilidad_aceptacion(double fitness_actual, double fitness_vecino, double temperatura) {
    if (fitness_vecino < fitness_actual)
        return 1.0;
    return exp((fitness_actual - fitness_vecino) / temperatura);
}

void liberar_solucion(Solucion *actual) {
    if (actual != NULL) {
        free(actual->ruta.puntos);
        free(actual);
    }
}

// Guarda solución en formato CSV
void guardar_solucion_csv(const char *nombre_archivo, Solucion *sol, int num_puntos) {
    if (sol == NULL || sol->ruta.puntos == NULL) {
        fprintf(stderr, "guardar_solucion_csv: solución o puntos NULL\n");
        return;
    }
    FILE *f = fopen(nombre_archivo, "w");
    if (!f) {
        perror("Error al abrir archivo para guardar solución");
        return;
    }
    // Cabecera: solución y bordes
    fprintf(f, "x_central,y_central,x_border1,y_border1,x_border2,y_border2\n");
    for (int i = 0; i < num_puntos; ++i) {
        Punto *p = &sol->ruta.puntos[i];
        fprintf(f,
                "%.18e,%.18e,%.18e,%.18e,%.18e,%.18e\n",
                p->x, p->y,
                p->x_border1, p->y_border1,
                p->x_border2, p->y_border2);
    }
    fclose(f);
}

// Lee puntos desde archivo CSV
void leer_puntos(Circuito *circuito, char* nombre_archivo, int *num_puntos) {
    FILE *archivo = fopen(nombre_archivo, "r");
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

    char linea[256];
    while (fgets(linea, sizeof(linea), archivo)) {
        Punto *temp = realloc(circuito->puntos, (*num_puntos + 1) * sizeof(Punto));
        if (temp == NULL) {
            perror("Error al asignar memoria");
            fclose(archivo);
            free(circuito->puntos);
            exit(EXIT_FAILURE);
        }
        circuito->puntos = temp;
        
        int campos_leidos = sscanf(
            linea, 
            "%lf,%lf,%lf,%lf,%lf,%lf", 
            &circuito->puntos[*num_puntos].x_central,
            &circuito->puntos[*num_puntos].y_central,
            &circuito->puntos[*num_puntos].x_border1,
            &circuito->puntos[*num_puntos].y_border1,
            &circuito->puntos[*num_puntos].x_border2,
            &circuito->puntos[*num_puntos].y_border2
        );
        
        if (campos_leidos != 6) {
            fprintf(stderr, "Error en el formato de la línea: %s", linea);
            fclose(archivo);
            free(circuito->puntos);
            exit(EXIT_FAILURE);
        }
        
        // Inicializar posición actual con el centro
        circuito->puntos[*num_puntos].x = circuito->puntos[*num_puntos].x_central;
        circuito->puntos[*num_puntos].y = circuito->puntos[*num_puntos].y_central;
        
        (*num_puntos)++;
    }
    fclose(archivo);
}

// Imprime puntos para depuración
void imprimir_puntos(Circuito *circuito, int num_puntos) {
    for (int i = 0; i < num_puntos; i++) {
        Punto *p = &circuito->puntos[i];
        printf("Punto %d:\n", i + 1);
        printf("  Actual:    (%.2f, %.2f)\n", p->x, p->y);
        printf("  Central:   (%.2f, %.2f)\n", p->x_central, p->y_central);
        printf("  Border1:   (%.2f, %.2f)\n", p->x_border1, p->y_border1);
        printf("  Border2:   (%.2f, %.2f)\n", p->x_border2, p->y_border2);
    }
}

int main() {
    // Iniciamos la medición del tiempo
    time_t inicio = time(NULL);
    srand(time(NULL));

    // Definición de variables
    int num_puntos = 0;
    double temperatura_inicial;
    double temperatura_final = 0.000001;
    //int num_generaciones = 1000000;
    //int num_generaciones = 10000;
    int num_generaciones = 500;

    // Parámetros adaptativos
    const int max_neighbours = 6000;
    const int max_successes = (int)(0.1 * max_neighbours);

    // Inicialización de la ruta actual
    Solucion *actual = (Solucion*)malloc(sizeof(Solucion));
    actual->ruta.puntos = NULL;

    // Nombre del archivo con las distancias
    char *nombre_archivo = "C:\\Users\\albsa\\Desktop\\Servicio Social\\Enfoque_Puntos_Pista\\CSV_Daniel\\Pista_Dani.csv";

    // Leer los puntos del archivo y guardar en la ruta actual
    leer_puntos(&actual->ruta, nombre_archivo, &num_puntos);

    crear_individuo(&actual->ruta, num_puntos);
    actual->fitness = evaluar_fitness(&actual->ruta, num_puntos);

    // Imprimir los puntos leídos
    imprimir_puntos(&actual->ruta, num_puntos);

    // Inicialización de la mejor ruta
    Solucion *mejor = (Solucion*)malloc(sizeof(Solucion));
    mejor->ruta.puntos = (Punto*)malloc(num_puntos * sizeof(Punto));
    memcpy(mejor->ruta.puntos, actual->ruta.puntos, num_puntos * sizeof(Punto));
    mejor->fitness = actual->fitness;

    // Imprimir el fitness inicial
    printf("Fitness inicial: %.2f grados\n", actual->fitness);

    // Rectificar el circuito inicial
    rectificar_circuito(&actual->ruta, num_puntos);
    actual->fitness = evaluar_fitness(&actual->ruta, num_puntos);
    printf("Fitness después de rectificar: %.2f grados\n", actual->fitness);

    // Calcular la temperatura inicial
    double suma = 0, suma_cuadrados = 0;
    for (int i = 0; i < 100; i++) {
        // Crear copia para vecino
        Punto *vecino_puntos = (Punto*)malloc(num_puntos * sizeof(Punto));
        memcpy(vecino_puntos, actual->ruta.puntos, num_puntos * sizeof(Punto));
        Circuito vecino = { vecino_puntos };
        
        rectificar_circuito(&vecino, num_puntos);
        double fit_vecino = evaluar_fitness(&vecino, num_puntos);
        
        suma += fit_vecino;
        suma_cuadrados += fit_vecino * fit_vecino;
        free(vecino_puntos);
    }

    double desviacion = sqrt((suma_cuadrados - suma * suma / 100) / 99);
    temperatura_inicial = desviacion;
    printf("Temperatura inicial: %.6f\n", temperatura_inicial);
    
    double temperatura = temperatura_inicial;
    // Ciclo principal de recocido simulado
    for (int iter = 1; iter <= num_generaciones && temperatura > temperatura_final; iter++) {
        temperatura *= 0.9999;  // Enfriamiento exponencial

        int neighbours = 0;
        int successes = 0;

        // Fase de equilibrio
        while (neighbours < max_neighbours && successes < max_successes) {
            // Crear copia profunda para vecino
            Punto *vecino_puntos = (Punto*)malloc(num_puntos * sizeof(Punto));
            memcpy(vecino_puntos, actual->ruta.puntos, num_puntos * sizeof(Punto));
            Circuito vecino_circuito = { vecino_puntos };
            
            rectificar_circuito(&vecino_circuito, num_puntos);
            double fit_vecino = evaluar_fitness(&vecino_circuito, num_puntos);

            double prob_acept = probabilidad_aceptacion(actual->fitness, fit_vecino, temperatura);
            if (prob_acept > rand_double_0_1()) {
                // Aceptar vecino
                free(actual->ruta.puntos);
                actual->ruta.puntos = vecino_puntos;
                actual->fitness = fit_vecino;
                successes++;

                // Actualizar mejor solución
                if (actual->fitness < mejor->fitness) {
                    free(mejor->ruta.puntos);
                    mejor->ruta.puntos = (Punto*)malloc(num_puntos * sizeof(Punto));
                    memcpy(mejor->ruta.puntos, actual->ruta.puntos, num_puntos * sizeof(Punto));
                    mejor->fitness = actual->fitness;
                }
            } else {
                // Rechazar vecino
                free(vecino_puntos);
            }
            neighbours++;
        }

        // Mostrar progreso
        printf("Iter %d | Temp = %.6f | Neighbors = %d | Successes = %d | Mejor = %.2f grados\n",
               iter, temperatura, neighbours, successes, mejor->fitness);
    }

    // Guardar y liberar recursos
    guardar_solucion_csv("Solucion_Angulo.csv", mejor, num_puntos);
    liberar_solucion(actual);
    liberar_solucion(mejor);

    return 0;
}
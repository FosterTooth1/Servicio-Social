#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>

#define M_PI 3.14159265358979323846

// --- NUEVAS DEFINICIONES PARA CONDICIONES DE PARADA ---
#define TARGET_FITNESS 0.1 // Ejemplo: Ajusta este valor según lo que consideres una solución "buena".
#define MAX_ITER_WITHOUT_IMPROVEMENT 1000 // Número máximo de iteraciones sin una mejora significativa antes de detenerse.
// ----------------------------------------------------

// Estructura de punto con coordenadas centrales y bordes
typedef struct {
    double x;              // Posición actual (optimizada)
    double y;
    double x_central;      // Centro de la pista
    double y_central;
    double x_border1;      // Borde 1
    double y_border1;
    double x_border2;      // Borde 2
    double y_border2;
    bool is_curve;         // Indica si es una curva
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
void determinar_limites_interno_externo(const Punto *p, double *xi, double *yi, double *xe, double *ye);
double calcular_distancia_dos_puntos(const Punto *p1, const Punto *p2);
double calcular_radio_curvatura(const Punto *p1, const Punto *p2, const Punto *p3);
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
    *xi = p->x_border2;  // Borde interno
    *yi = p->y_border2;
    *xe = p->x_border1;  // Borde externo
    *ye = p->y_border1;
}

// Crea individuo inicial
void crear_individuo(Circuito *circuito, int num_puntos) {
    if (circuito == NULL || circuito->puntos == NULL) {
        fprintf(stderr, "crear_individuo: circuito o puntos NULL\n");
        return;
    }

    for (int i = 0; i < num_puntos; i++) {
        Punto *p = &circuito->puntos[i];
        
        if (!p->is_curve) {
            // Rectas: posición fija en centro
            p->x = p->x_central;
            p->y = p->y_central;
        } else {
            // Curvas: interpolación aleatoria entre bordes
            double xi, yi, xe, ye;
            determinar_limites_interno_externo(p, &xi, &yi, &xe, &ye);

            double dx = xe - xi;
            double dy = ye - yi;
            double dist2 = dx*dx + dy*dy;
            
            if (dist2 < 1e-12) {
                p->x = xi;
                p->y = yi;
                continue;
            }

            double t = rand_double_0_1();
            p->x = xi + t * dx;
            p->y = yi + t * dy;
        }
    }
}

// Calcula la distancia entre dos puntos
double calcular_distancia_dos_puntos(const Punto *p1, const Punto *p2) {
    double dx = p2->x - p1->x;
    double dy = p2->y - p1->y;
    return sqrt(dx * dx + dy * dy);
}

// Calcula radio de curvatura usando 3 puntos (fórmula del circuncírculo)
double calcular_radio_curvatura(const Punto *p1, const Punto *p2, const Punto *p3) {
    double a = calcular_distancia_dos_puntos(p2, p3); // |P2-P3|
    double b = calcular_distancia_dos_puntos(p1, p3); // |P1-P3|
    double c = calcular_distancia_dos_puntos(p1, p2); // |P1-P2|
    
    // Área del triángulo (fórmula de Gauss)
    double area = 0.5 * fabs(
        p1->x*(p2->y - p3->y) +
        p2->x*(p3->y - p1->y) +
        p3->x*(p1->y - p2->y)
    );
    
    // Evitar división por cero (puntos colineales)
    if (area < 1e-8) return 1e10; // Valor grande para rectas
    
    return (a * b * c) / (4 * area); // Radio de curvatura
}

// Rectifica el circuito usando parámetro t (solo en curvas)
void rectificar_circuito(Circuito *circuito, int num_puntos) {
    if (circuito == NULL || circuito->puntos == NULL) return;
    if (num_puntos < 3) return;

    const double Pm = 0.01;  // Probabilidad de modificación
    const int Nm = 100;      // Parámetro de distribución

    for (int i = 0; i < num_puntos - 2; ++i) {
        Punto *p2 = &circuito->puntos[i + 1];
        
        // Solo modificar puntos en curva
        if (!p2->is_curve) continue;

        // Determinar límites interno/externo para p2
        double xi, yi, xe, ye;
        determinar_limites_interno_externo(p2, &xi, &yi, &xe, &ye);

        // Vector entre bordes
        double vx = xe - xi;
        double vy = ye - yi;
        double len2 = vx*vx + vy*vy;
        if (len2 < 1e-12) continue;

        // Calcular t actual
        double wx = p2->x - xi;
        double wy = p2->y - yi;
        double t_actual = (wx * vx + wy * vy) / len2;
        if (t_actual < 0.0) t_actual = 0.0;
        if (t_actual > 1.0) t_actual = 1.0;

        // Radio de curvatura original
        double radio_orig = calcular_radio_curvatura(
            &circuito->puntos[i], 
            p2, 
            &circuito->puntos[i + 2]
        );

        double x_orig = p2->x;
        double y_orig = p2->y;

        // Intentar modificación con probabilidad Pm
        if (rand_double_0_1() <= Pm) {
            // Calcula deltaq_t
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

            // Actualizar posición provisional
            p2->x = xi + t_nuevo * vx;
            p2->y = yi + t_nuevo * vy;

            // Calcular radio nuevo
            double radio_new = calcular_radio_curvatura(
                &circuito->puntos[i], 
                p2, 
                &circuito->puntos[i + 2]
            );

            // Solo aceptar si mejora el radio de curvatura
            if (radio_new < radio_orig) { // Se acepta si el nuevo radio es menor (para minimizar 1/radio)
                p2->x = x_orig;
                p2->y = y_orig;
            }
        }
    }
}

// Evalúa fitness con nuevo criterio: centro en rectas y radio en curvas
double evaluar_fitness(Circuito *circuito, int num_puntos) {
    if (circuito == NULL || circuito->puntos == NULL) return 0.0;
    if (num_puntos < 3) return 0.0;
    
    double fitness_curvatura = 0.0;
    double fitness_centro = 0.0;

    // 1. Penalización por salirse del centro en RECTAS
    for (int i = 0; i < num_puntos; i++) {
        Punto *p = &circuito->puntos[i];
        if (!p->is_curve) {
            double dx = p->x - p->x_central;
            double dy = p->y - p->y_central;
            fitness_centro += sqrt(dx*dx + dy*dy);
        }
    }

    // 2. Minimizar curvatura en CURVAS (maximizar radio)
    for (int i = 1; i < num_puntos - 1; i++) {
        Punto *p = &circuito->puntos[i];
        if (p->is_curve) {
            double radio = calcular_radio_curvatura(
                &circuito->puntos[i-1],
                p,
                &circuito->puntos[i+1]
            );
            // Queremos MAXIMIZAR el radio -> MINIMIZAR 1/radio
            if (radio < 1e-9) radio = 1e-9; // Evitar división por cero
            fitness_curvatura += 1.0 / radio;
        }
    }

    // Ponderación final (mayor peso a curvaturas)
    return (0.3 * fitness_centro) + (0.7 * fitness_curvatura);
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

// ... (previous code)

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
    // --- CAMBIO AQUI: Nombres de las columnas para que coincidan con tu script Python ---
    fprintf(f, "x_central,y_central,x_border1,y_border1,x_border2,y_border2,is_curve\n");
    // ----------------------------------------------------------------------------------
    for (int i = 0; i < num_puntos; ++i) {
        Punto *p = &sol->ruta.puntos[i];
        fprintf(f,
                "%.18e,%.18e,%.18e,%.18e,%.18e,%.18e,%d\n",
                p->x, p->y, // Esto ahora se mapea a x_central, y_central en el encabezado
                p->x_border1, p->y_border1,
                p->x_border2, p->y_border2,
                p->is_curve ? 1 : 0);
    }
    fclose(f);
}

// ... (rest of the code)

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
        double x_central, y_central, x_border1, y_border1, x_border2, y_border2;
        int is_curve_int;
        int campos_leidos = sscanf(
            linea, 
            "%lf,%lf,%lf,%lf,%lf,%lf,%d", 
            &x_central,
            &y_central,
            &x_border1,
            &y_border1,
            &x_border2,
            &y_border2,
            &is_curve_int
        );
        
        if (campos_leidos != 7) {
            fprintf(stderr, "Error en el formato de la línea: %s", linea);
            fclose(archivo);
            free(circuito->puntos);
            exit(EXIT_FAILURE);
        }

        Punto *temp = realloc(circuito->puntos, (*num_puntos + 1) * sizeof(Punto));
        if (temp == NULL) {
            perror("Error al asignar memoria");
            fclose(archivo);
            free(circuito->puntos);
            exit(EXIT_FAILURE);
        }
        circuito->puntos = temp;

        circuito->puntos[*num_puntos].x_central = x_central;
        circuito->puntos[*num_puntos].y_central = y_central;
        circuito->puntos[*num_puntos].x_border1 = x_border1;
        circuito->puntos[*num_puntos].y_border1 = y_border1;
        circuito->puntos[*num_puntos].x_border2 = x_border2;
        circuito->puntos[*num_puntos].y_border2 = y_border2;
        circuito->puntos[*num_puntos].is_curve = (bool)is_curve_int;

        // Inicializar posición actual con el centro
        circuito->puntos[*num_puntos].x = x_central;
        circuito->puntos[*num_puntos].y = y_central;

        (*num_puntos)++;
    }
    fclose(archivo);
}

// Imprime puntos para depuración
void imprimir_puntos(Circuito *circuito, int num_puntos) {
    for (int i = 0; i < num_puntos; i++) {
        Punto *p = &circuito->puntos[i];
        printf("Punto %d: %s\n", i + 1, p->is_curve ? "CURVA" : "RECTA");
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
    int num_generaciones = 10000;

    // Parámetros adaptativos
    const int max_neighbours = 6000;
    const int max_successes = (int)(0.1 * max_neighbours);

    // Inicialización de la ruta actual
    Solucion *actual = (Solucion*)malloc(sizeof(Solucion));
    actual->ruta.puntos = NULL;

    // Nombre del archivo con las distancias
    // ¡IMPORTANTE!: Cambia esta ruta a la ubicación de tu archivo CSV
    char *nombre_archivo = "C:/Users/anyax/Desktop/SS CODIGO/Servicio-Social/YAX_PRUEBAS/PruebaHermanos.csv";

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
    printf("Fitness inicial: %.2f\n", actual->fitness);

    // Rectificar el circuito inicial
    rectificar_circuito(&actual->ruta, num_puntos);
    actual->fitness = evaluar_fitness(&actual->ruta, num_puntos);
    printf("Fitness después de rectificar: %.2f\n", actual->fitness);

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
    temperatura_inicial = desviacion * 1; // Ajuste de escala
    printf("Temperatura inicial: %.6f\n", temperatura_inicial);
    
    double temperatura = temperatura_inicial;

    // --- Variable para la condición de parada por no mejora ---
    int last_improvement_iter = 0; 
    // ---------------------------------------------------------

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
                    last_improvement_iter = iter; // Se actualizó la mejor, reseteamos el contador de no mejora.
                }
            } else {
                // Rechazar vecino
                free(vecino_puntos);
            }
            neighbours++;
        }

        // --- Verificación de condiciones de parada ---
        if (mejor->fitness <= TARGET_FITNESS) {
            printf("¡Parada anticipada: Fitness objetivo (%.2f) alcanzado!\n", TARGET_FITNESS);
            break; // Salir del bucle principal
        }

        if (iter - last_improvement_iter > MAX_ITER_WITHOUT_IMPROVEMENT) {
            printf("¡Parada anticipada: No hubo mejora significativa por %d iteraciones!\n", MAX_ITER_WITHOUT_IMPROVEMENT);
            break; // Salir del bucle principal
        }
        // --------------------------------------------

        // Mostrar progreso
        printf("Iter %d | Temp = %.6f | Neighbors = %d | Successes = %d | Mejor = %.2f\n",
               iter, temperatura, neighbours, successes, mejor->fitness);
    }

    // Guardar y liberar recursos
    guardar_solucion_csv("Solucion_Definitiva.csv", mejor, num_puntos);
    liberar_solucion(actual);
    liberar_solucion(mejor);

    printf("Optimización finalizada. Solución guardada en Solucion_Definitiva.csv\n");

    return 0;
}
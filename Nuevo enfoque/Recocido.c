#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include <math.h>

// Definición de la estructura de las coordenadas de los puntos junto con sus límites
typedef struct {
    double x;
    double y;
    double limite_x_externo;
    double limite_y_externo;
    double limite_x_interno;
    double limite_y_interno;
} Punto;    

// Estructura del circuito
typedef struct {
    Punto *puntos;
} Circuito;

// Estructura de la solución
typedef struct
{
    Circuito ruta;      // Ruta de la actual
    double fitness; // Fitness de la actual
} Solucion;


void leer_puntos(Circuito *Circuito, char* nombre_archivo, int *num_puntos);
void imprimir_puntos(Circuito *Circuito, int num_puntos);
void liberar_puntos(Circuito *Circuito);
double calcular_angulo_tres_puntos(Punto p1, Punto p2, Punto p3);


// Funciones principales del Recocido Simulado
// Libera la memoria usada para la actual
void liberar_solucion(Solucion *actual);
// Calcula el fitness de la ruta actual
double evaluar_fitness(Circuito *Circuito, int num_puntos);
// Genera un vecino de la ruta actual
void rectificar_circuito(Circuito *Circuito, int num_puntos);
// Calcula la probabilidad de aceptación de un nuevo vecino
double probabilidad_aceptacion(double fitness_actual, double fitness_vecino, double temperatura);

void leer_puntos(Circuito *Circuito, char* nombre_archivo, int *num_puntos) {
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
        Punto *temp = realloc(Circuito->puntos, (*num_puntos + 1) * sizeof(Punto));
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
            &Circuito->puntos[*num_puntos].x, 
            &Circuito->puntos[*num_puntos].y,
            &Circuito->puntos[*num_puntos].limite_x_externo,
            &Circuito->puntos[*num_puntos].limite_y_externo,
            &Circuito->puntos[*num_puntos].limite_x_interno,
            &Circuito->puntos[*num_puntos].limite_y_interno
        );
        
        if (campos_leidos != 6) {
            fprintf(stderr, "Error en el formato de la línea: %s", linea);
            fclose(archivo);
            free(Circuito->puntos);
            exit(EXIT_FAILURE);
        }
        
        (*num_puntos)++;
    }

    fclose(archivo);
}

void imprimir_puntos(Circuito *Circuito, int num_puntos) {
    for (int i = 0; i < num_puntos; i++) {
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

void rectificar_circuito(Circuito *Circuito, int num_puntos) {
    if (num_puntos < 3) return;

    const double Pm = 0.1;  // Probabilidad de modificación
    const int Nm = 100;

    for (int i = 0; i < num_puntos - 2; ++i) {
        Punto *p1 = &Circuito->puntos[i];
        Punto *p2 = &Circuito->puntos[i + 1];
        Punto *p3 = &Circuito->puntos[i + 2];
        
        // Ángulo original antes de modificar
        double angulo_original = calcular_angulo_tres_puntos(*p1, *p2, *p3);
        double x_original = p2->x;
        double y_original = p2->y;
        bool modificado = false;

        // Intentar modificar X (siempre que se cumpla Pm)
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

            double upper_y = fmax(p2->limite_y_externo, p2->limite_y_interno);
            double lower_y = fmin(p2->limite_y_externo, p2->limite_y_interno);
            double delta_y = fmin(upper_y - p2->y, p2->y - lower_y) / (upper_y - lower_y);
            r = (double)rand() / RAND_MAX;
            double deltaq_y;

            if (r <= 0.5) {
                deltaq_y = pow(2 * r + (1 - 2 * r) * pow(1 - delta_y, Nm + 1), 1.0 / (Nm + 1)) - 1;
            } else {
                deltaq_y = 1 - pow(2 * (1 - r) + 2 * (r - 0.5) * pow(1 - delta_y, Nm + 1), 1.0 / (Nm + 1));
            }

            p2->y += deltaq_y * (upper_y - lower_y);
            p2->y = fmax(lower_y, fmin(p2->y, upper_y));
            modificado = true;
        }

        // Verificar si el ángulo empeora después de modificar
        if (modificado) {
            double nuevo_angulo = calcular_angulo_tres_puntos(*p1, *p2, *p3);
            if (nuevo_angulo > angulo_original) {
                // Revertir cambios si el ángulo aumenta
                p2->x = x_original;
                p2->y = y_original;
            }
        }
    }
}

double evaluar_fitness(Circuito *Circuito, int num_puntos) {
    if (num_puntos < 3) return 0.0;

    double suma_angulos = 0.0;
    for (int i = 0; i < num_puntos - 2; ++i) {
        Punto p1 = Circuito->puntos[i];
        Punto p2 = Circuito->puntos[i + 1];
        Punto p3 = Circuito->puntos[i + 2];
        
        double angulo = calcular_angulo_tres_puntos(p1, p2, p3);
        suma_angulos += angulo;
    }
    return suma_angulos;
}

double probabilidad_aceptacion(double fitness_actual, double fitness_vecino, double temperatura)
{
    if (fitness_vecino < fitness_actual)
        return 1.0;
    return exp((fitness_actual - fitness_vecino) / temperatura);
}

void liberar_solucion(Solucion *actual) {
    if (actual != NULL) {
        free(actual->ruta.puntos);  // Liberar el array de puntos
        free(actual);               // Liberar la estructura Solucion
    }
}


int main()
{
    // Iniciamos la medición del tiempo
    time_t inicio = time(NULL);
    srand(time(NULL));

    // Definición de variables
    int num_puntos = 0;
    double temperatura_inicial;
    double temperatura_final = 0.000000001;
    int num_generaciones = 100000;

    // Parámetros adaptativos
    const int max_neighbours = 2000;
    const int max_successes = (int)(0.1 * max_neighbours);

    // Inicialización de la ruta actual
    Solucion *actual = (Solucion*)malloc(sizeof(Solucion));
    actual->ruta.puntos = NULL;

    // Nombre del archivo con las distancias
    char *nombre_archivo = "pista_escalada.csv";

    // Leer los puntos del archivo y guardar en la ruta actual
    leer_puntos(&actual->ruta, nombre_archivo, &num_puntos);

    // Guardar el fitness de la ruta actual
    actual->fitness = evaluar_fitness(&actual->ruta, num_puntos);

    // Imprimir los puntos leídos
    imprimir_puntos(&actual->ruta, num_puntos);

    // Inicialización de la mejor ruta
    Solucion *mejor = (Solucion*)malloc(sizeof(Solucion));
    mejor->ruta.puntos = NULL;

    // La mejor ruta es la misma que la actual
    mejor->ruta.puntos = (Punto*)malloc(num_puntos * sizeof(Punto));
    memcpy(mejor->ruta.puntos, actual->ruta.puntos, num_puntos * sizeof(Punto));
    mejor->fitness = actual->fitness;

    // Imprimir el fitness inicial
    printf("Fitness inicial: %.2f grados\n", actual->fitness);

    // Rectificar el circuito inicial
    rectificar_circuito(&actual->ruta, num_puntos);

    // Imprimir el fitness después de la rectificación
    actual->fitness = evaluar_fitness(&actual->ruta, num_puntos);
    printf("Fitness después de rectificar: %.2f grados\n", actual->fitness);



    // Liberar memoria de la ruta
    liberar_solucion(actual);
    

    return 0;
}


/*

    int *vecino = malloc(longitud_ruta * sizeof(int));

    double suma = 0, suma_cuadrados = 0;
    for (int i = 0; i < 100; i++)
    {
        generar_vecino(actual->ruta, vecino, longitud_ruta);
        // Aplicar la heurística de remoción de abruptos al vecino
        heuristica_abruptos(vecino, longitud_ruta, m, distancias);
        double fit = calcular_fitness(vecino, distancias, longitud_ruta);
        suma += fit;
        suma_cuadrados += fit * fit;
    }
    double desviacion = sqrt((suma_cuadrados - suma * suma / 100) / 99);
    temperatura_inicial = desviacion; // Ajuste empírico

    // Calculo de la temperatura inicial
    // Usando un porcentaje de la mejor solución
    // temperatura_inicial = mejor->fitness * 0.3;

    double temperatura = temperatura_inicial;

    // Ciclo principal de recocido con enfriamiento logarítmico (Béltsman)
    for (int iter = 1; iter <= num_generaciones && temperatura > temperatura_final; iter++)
    {
        // Enfriamiento logarítmico de Béltsman:
        // T_k = T0 / ln(k + 1)
        temperatura = temperatura_inicial / log(iter + 1.0);

        int neighbours = 0;
        int successes = 0;

        // Fase de equilibrio: hasta max_neighbours o max_successes
        while (neighbours < max_neighbours && successes < max_successes)
        {
            generar_vecino(actual->ruta, vecino, longitud_ruta);
            double fit_vecino = calcular_fitness(vecino, distancias, longitud_ruta);

            if (probabilidad_aceptacion(actual->fitness, fit_vecino, temperatura) > ((double)rand() / RAND_MAX))
            {
                // Aceptamos el vecino
                memcpy(actual->ruta, vecino, longitud_ruta * sizeof(int));
                actual->fitness = fit_vecino;
                successes++;

                // Actualizamos el mejor si procede
                if (actual->fitness < mejor->fitness)
                {
                    memcpy(mejor->ruta, actual->ruta, longitud_ruta * sizeof(int));
                    mejor->fitness = actual->fitness;
                }
            }

            neighbours++;
        }

        // Aplicar heurística de remoción de abruptos tras cada enfriamiento
        heuristica_abruptos(actual->ruta,
                            longitud_ruta,
                            m,
                            distancias);
        actual->fitness = calcular_fitness(actual->ruta,
                                           distancias,
                                           longitud_ruta);

        // (Opcional) Mostrar info de cada paso de enfriamiento
        printf("Iter %3d | Temp = %.6f | Neighbors = %3d | Successes = %2d | Mejor = %.2f\n",
               iter, temperatura, neighbours, successes, mejor->fitness);
    }

    // Mostrar tiempo de ejecución
    time_t fin = time(NULL);
    double tiempo_ejecucion = difftime(fin, inicio);
    printf("Tiempo de ejecución: %.2f segundos\n", tiempo_ejecucion);

    // Liberar memoria y mostrar resultados
    free(vecino);
    printf("\nMejor ruta encontrada (%.2f km):\n", mejor->fitness);
    for (int i = 0; i < longitud_ruta; i++)
    {
        printf("%s -> ", nombres_ciudades[mejor->ruta[i]]);
    }
    printf("%s\n", nombres_ciudades[mejor->ruta[0]]);

    liberar_solucion(actual);
    liberar_solucion(actual);
    liberar_solucion(mejor);
    for (int i = 0; i < longitud_ruta; i++)
    {
        free(distancias[i]);
    }
    free(distancias);
    return 0;
}

*/
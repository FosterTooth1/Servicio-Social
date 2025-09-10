#include "Biblioteca.h"

int main(int argc, char **argv)
{
    // Iniciamos la medición del tiempo
    time_t inicio = time(NULL);

    // Parámetros del algoritmo genético
    srand(time(NULL));
    int tamano_poblacion = 5000; // Tamaño de la población
    double delta_t = 0.01; // Paso de tiempo
    double tiempo_test = 8.0; // Tiempo total de simulación
    int longitud_genotipo = tiempo_test / delta_t; // Longitud del genotipo (numero de variables)
    int num_generaciones = 300; // Número de generaciones
    int num_competidores = 50; // Número de competidores en la selección por torneo
    double limite_inferior = -0.5; // Límite inferior para los valores del genotipo
    double limite_superior = 0.5; // Límite superior para los valores del genotipo
    double probabilidad_mutacion = 0.3; // Probabilidad de mutación
    double probabilidad_cruce = 0.9; // Probabilidad de cruce
    double B = 0.2; // Ancho del robot en metros
    double nc = 10; // SBX (0-20) //Mas alto es mas explotacion
    double nm = 60; // Polinomial (20-100)
    double distancia_umbral = 0.1; // Distancia umbral para considerar que se alcanzó un punto objetivo
    
    // Definir puntos objetivo (ejemplo)
    PuntoObjetivo objetivos[] = {
         {2.0, 3.0},
         {5.0, 7.0},
         {8.0, 10.0}};
    int num_objetivos = sizeof(objetivos) / sizeof(objetivos[0]);
    printf("%d", num_objetivos);

    // Inicializamos la población
    poblacion *Poblacion = inicializar_poblacion(tamano_poblacion, longitud_genotipo);
    poblacion *padres = inicializar_poblacion(tamano_poblacion, longitud_genotipo);
    poblacion *hijos = inicializar_poblacion(tamano_poblacion, longitud_genotipo);

    // Creamos valores aleatorios de genotipo para cada individuo de la población
    crear_poblacion(Poblacion, longitud_genotipo, delta_t, B, limite_inferior, limite_superior, objetivos, num_objetivos, distancia_umbral);
    
    // Imprimir poblacion
    // imprimir_poblacion(Poblacion, longitud_genotipo);

    // Evaluamos la población
    evaluar_poblacion(Poblacion, longitud_genotipo, delta_t, B, objetivos, num_objetivos, distancia_umbral);

    // Ordenamos la población
    ordenar_poblacion(Poblacion);

    // Inicializamos el mejor individuo
    individuo *Mejor_Individuo = (individuo *)malloc(sizeof(individuo));
    Mejor_Individuo->genotipo_izquierdo = (double *)malloc(longitud_genotipo * sizeof(double));
    Mejor_Individuo->genotipo_derecho = (double *)malloc(longitud_genotipo * sizeof(double));

    // Copiamos el mejor individuo de la población a Mejor_Individuo
    for (int i = 0; i < longitud_genotipo; i++)
    {
        Mejor_Individuo->genotipo_izquierdo[i] = Poblacion->individuos[0].genotipo_izquierdo[i];
        Mejor_Individuo->genotipo_derecho[i] = Poblacion->individuos[0].genotipo_derecho[i];
    }

    // Copiamos el fitness del mejor individuo
    Mejor_Individuo->fitness = Poblacion->individuos[0].fitness;
    /*
    // Ejecutamos el algoritmo genético
    for (int generacion = 0; generacion < num_generaciones; generacion++)
    {
        // Seleccionamos a los padres
        seleccionar_padres_torneo(Poblacion, padres, num_competidores, longitud_genotipo);

        // Cruzamos a los padres
        cruzar_individuos(padres, hijos, tamano_poblacion, longitud_genotipo, probabilidad_cruce, delta_t, B, limite_inferior, limite_superior, nc, objetivos, num_objetivos, distancia_umbral);

        // Mutamos a los hijos
        for (int i = 0; i < tamano_poblacion; i++)
        {
            mutar_individuo(&hijos->individuos[i], probabilidad_mutacion, longitud_genotipo, limite_inferior, limite_superior, nm);
        }

        // Reemplazamos la población actual con los hijos
        actualizar_poblacion(&Poblacion, hijos, longitud_genotipo);

        // Evaluamos a los hijos
        evaluar_poblacion(Poblacion, longitud_genotipo, delta_t, B, objetivos, num_objetivos, distancia_umbral);
        ordenar_poblacion(Poblacion);

        // Actualizamos al mejor individuo si es necesario
        if (Poblacion->individuos[0].fitness < Mejor_Individuo->fitness)
        {
            for (int i = 0; i < longitud_genotipo; i++)
            {
                Mejor_Individuo->genotipo_izquierdo[i] = Poblacion->individuos[0].genotipo_izquierdo[i];
                Mejor_Individuo->genotipo_derecho[i] = Poblacion->individuos[0].genotipo_derecho[i];
            }
            Mejor_Individuo->fitness = Poblacion->individuos[0].fitness;
        }
    }

    */

    // Imprimimos al mejor individuo
    printf("Fitness del mejor individuo: %Lf\n", Mejor_Individuo->fitness);

    // --- Guardar la trayectoria del mejor individuo en un archivo CSV ---
    FILE *fp = fopen("trayectoria.csv", "w");
    if (fp == NULL)
    {
        printf("Error al abrir el archivo CSV.\n");
        exit(1);
    }
    // Escribir encabezado
    fprintf(fp, "time,x,y\n");

    // Condiciones iniciales para la simulación de la trayectoria
    double x_pos = 0.0, y_pos = 0.0, phi_pos = 0.0;
    double vl = 0.0, vr = 0.0;
    double t = 0.0;
    int objetivo_actual = 0;
    for (int j = 0; j < longitud_genotipo; j++)
    {
        double u1 = Mejor_Individuo->genotipo_izquierdo[j];
        double u2 = Mejor_Individuo->genotipo_derecho[j];
        runge_kutta(delta_t, &x_pos, &y_pos, &phi_pos, &vl, &vr, &u1, &u2, B);
        t += delta_t;

        // Verificar si se alcanzó el objetivo actual
        double dx = objetivos[objetivo_actual].x - x_pos;
        double dy = objetivos[objetivo_actual].y - y_pos;
        if (sqrt(dx * dx + dy * dy) < distancia_umbral && objetivo_actual < num_objetivos - 1)
        {
            objetivo_actual++;
        }

        fprintf(fp, "%.4f,%.4f,%.4f\n", t, x_pos, y_pos);
    }
    printf("Trayectoria guardada en 'trayectoria.csv'.\n");

    // Liberamos la memoria de todos los elementos
    liberar_poblacion(Poblacion);
    liberar_poblacion(padres);
    liberar_poblacion(hijos);
    free(Mejor_Individuo->genotipo_izquierdo);
    Mejor_Individuo->genotipo_izquierdo = NULL;
    free(Mejor_Individuo->genotipo_derecho);
    Mejor_Individuo->genotipo_derecho = NULL;
    free(Mejor_Individuo);

    // Finalizamos la medición del tiempo
    time_t fin = time(NULL);

    // Imprimimos el tiempo de ejecución
    double tiempo_ejecucion = difftime(fin, inicio);
    printf("Tiempo de ejecucion: %.2f segundos\n", tiempo_ejecucion);

    return 0;
}
#include "Biblioteca.h"

int main(int argc, char** argv){
    // Iniciamos la medición del tiempo
    time_t inicio = time(NULL);

    // Parámetros del algoritmo genético
    srand(time(NULL));
    int tamano_poblacion = 1000;
    double delta_t= 0.01;
    double tiempo_test= 7.0;
    int longitud_genotipo = tiempo_test/delta_t;
    int num_generaciones  = 100;
    int num_competidores  = 2;
    double limite_inferior = -0.5;
    double limite_superior = 0.5;
    double probabilidad_mutacion = 0.15;
    double probabilidad_cruce = 0.9;
    double B = 0.2;

    // Inicializamos la población
    poblacion *Poblacion = inicializar_poblacion(tamano_poblacion, longitud_genotipo);
    poblacion *padres = inicializar_poblacion(tamano_poblacion, longitud_genotipo);    
    poblacion *hijos = inicializar_poblacion(tamano_poblacion, longitud_genotipo);

    // Creamos valores aleatorios de genotipo para cada individuo de la población
    crear_poblacion(Poblacion, longitud_genotipo, delta_t, B);

    //Imprimir poblacion
    //imprimir_poblacion(Poblacion, longitud_genotipo);

    // Evaluamos la población
    evaluar_poblacion(Poblacion, longitud_genotipo, delta_t, B);

    // Ordenamos la población
    ordenar_poblacion(Poblacion);

    // Inicializamos el mejor individuo
    individuo *Mejor_Individuo = (individuo *)malloc(sizeof(individuo));
    Mejor_Individuo->genotipo_izquierdo = (double *)malloc(longitud_genotipo * sizeof(double));
    Mejor_Individuo->genotipo_derecho = (double *)malloc(longitud_genotipo * sizeof(double));

    // Copiamos el mejor individuo de la población a Mejor_Individuo
    for (int i = 0; i < longitud_genotipo; i++) {
        Mejor_Individuo->genotipo_izquierdo[i] = Poblacion->individuos[0].genotipo_izquierdo[i];
        Mejor_Individuo->genotipo_derecho[i] = Poblacion->individuos[0].genotipo_derecho[i];
    }

    // Copiamos el fitness del mejor individuo
    Mejor_Individuo->fitness = Poblacion->individuos[0].fitness;
    
    // Ejecutamos el algoritmo genético
    for(int generacion=0; generacion<num_generaciones; generacion++){
        // Seleccionamos a los padres
        seleccionar_padres_torneo(Poblacion, padres, num_competidores, longitud_genotipo);

        // Cruzamos a los padres
        cruzar_individuos(padres, hijos, tamano_poblacion, longitud_genotipo, probabilidad_cruce, delta_t, B);

        // Mutamos a los hijos
        for (int i = 0; i < tamano_poblacion; i++) {
            mutar_individuo(&hijos->individuos[i], probabilidad_mutacion, longitud_genotipo);
        }

        //Reemplazamos la población actual con los hijos
        actualizar_poblacion(&Poblacion, hijos, longitud_genotipo);

        // Evaluamos a los hijos
        evaluar_poblacion(Poblacion, longitud_genotipo, delta_t, B);
        ordenar_poblacion(Poblacion);

        // Actualizamos al mejor individuo si es necesario
        if (Poblacion->individuos[0].fitness < Mejor_Individuo->fitness) {
        for (int i = 0; i < longitud_genotipo; i++) {
            Mejor_Individuo->genotipo_izquierdo[i] = Poblacion->individuos[0].genotipo_izquierdo[i];
            Mejor_Individuo->genotipo_derecho[i] = Poblacion->individuos[0].genotipo_derecho[i];
        }
        Mejor_Individuo->fitness = Poblacion->individuos[0].fitness;
        }
    }

    // Imprimimos al mejor individuo
    printf("Fitness del mejor individuo: %Lf\n", Mejor_Individuo->fitness);

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
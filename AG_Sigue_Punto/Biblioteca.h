#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<time.h>

// Estructuras
// Estructura para un individuon (Almacena el genotipo izquierdo y dreceho del robot y el fitness)
typedef struct{
    float *genotipo_izquierdo;
    float *genotipo_derecho;
    float fitness;
}individuo;

// Estructura para una población (Almacena un arreglo de individuos y su tamaño)
typedef struct{
    individuo *individuos;
    int tamano;
}poblacion;

// Estructura para ordenar distancias (Almacena la distancia y el índice)(Usado en la heurística de remoción de abruptos)
typedef struct {
    float distancia;
    int indice;
} DistanciaOrdenada;

//Funciones principales del algoritmo genético
//Asigna memoria para una población
poblacion *inicializar_poblacion(int tamano, int longitud_genotipo);
//Crea valores aleatorios para los genotipos de los individuos de la poblacion
void crear_poblacion(poblacion *poblacion, int longitud_genotipo);
//Evalúa a la población
void evaluar_poblacion(poblacion *poblacion, int longitud_genotipo);
//Evalúa a un individuo
float evaluar_individuo(float*u1, float *u2, int longitud_genotipo);
//Ordena a la población de acuerdo a su fitness mediante el algoritmo de introsort
void ordenar_poblacion(poblacion *poblacion);
//Selecciona a los padres de la población mediante un torneo de fitness
void seleccionar_padres_torneo(poblacion *Poblacion, poblacion *padres, int num_competidores, int longitud_genotipo);
//Cruza a los padres para generar a los hijos dependiendo de una probabilidad de cruce
void cruzar_individuos(poblacion *padres, poblacion *hijos, int num_pob, int longitud_genotipo, float probabilidad_cruce);
//Muta a un individuo dependiendo de una probabilidad de mutación
void mutar_individuo(individuo *individuo, float probabilidad_mutacion, int longitud_genotipo);
//Actualiza a la población con los nuevos individuos (hijos)
void actualizar_poblacion(poblacion **destino, poblacion *origen, int longitud_genotipo);
//Libera la memoria usada para la población
void liberar_poblacion(poblacion *poblacion);


//Funciones auxiliares del cruzamiento
// Función SBX para un solo gen (una variable)
// Recibe el valor del gen de cada padre, y genera dos hijos
void sbx_crossover(float parent1, float parent2, float *child1, float *child2,
    float eta, float lower_bound, float upper_bound);

//Funciones auxiliares de ordenamiento
// Introsort es un algoritmo de ordenamiento híbrido que combina QuickSort, HeapSort e InsertionSort
void introsort_util(individuo *arr, int *profundidad_max, int inicio, int fin);
//Calcula el logaritmo base 2 de un número para medir la profundidad de recursividad que puede alcanzar QuickSort
int log2_suelo(int n);
//Particiona el arreglo para el QuickSort (Funcion auxiliar de Introsort en especifico para el QuickSort)
int particion(individuo *arr, int bajo, int alto);
//Calcula la mediana de tres elementos (Funcion auxiliar de Introsort en especifico para el QuickSort)
int mediana_de_tres(individuo *arr, int a, int b, int c);
//Intercambia dos individuos (Funcion auxiliar de Introsort en especifico para el QuickSort)
void intercambiar_individuos(individuo *a, individuo *b);
//Insertion sort es un algoritmo de ordenamiento simple y eficiente para arreglos pequeños
void insertion_sort(individuo *arr, int izquierda, int derecha);
//Heapsort es un algoritmo de ordenamiento basado en árboles binarios
void heapsort(individuo *arr, int n);
//Heapify es una función auxiliar para heapsort
void heapify(individuo *arr, int n, int i);

//Funciones auxiliares de manipulación de arreglos (Usadas en la heurística de remoción de abruptos)
//Compara dos distancias para ordenarlas
int comparar_distancias(const void* a, const void* b);
//Inserta un elemento en una posición específica del arreglo
void insertar_en_posicion(int* array, int longitud, int elemento, int posicion);
//Elimina un elemento de una posición específica del arreglo
void eliminar_de_posicion(int* array, int longitud, int posicion);

////////////Runge Kutta

void runge_kutta(float delta_t, float* condicion_inicial_x, float* condicion_inicial_y, float* condicion_inicial_phi, float* condicion_inicial_vl, float* condicion_inicial_vr, float* condicion_inicial_u1, float* condicion_inicial_u2, float B);
float x(float vl, float vr, float phi);
float y(float vl, float vr, float phi);
float phi(float vl, float vr, float B);
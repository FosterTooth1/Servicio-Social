#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<time.h>
#include<math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Estructuras
// Estructura para un individuon (Almacena el genotipo izquierdo y dreceho del robot y el fitness)
typedef struct{
    double *genotipo_izquierdo;
    double *genotipo_derecho;
    double fitness;
}individuo;

// Estructura para una población (Almacena un arreglo de individuos y su tamaño)
typedef struct{
    individuo *individuos;
    int tamano;
}poblacion;

//Funciones principales del algoritmo genético
//Asigna memoria para una población
poblacion *inicializar_poblacion(int tamano, int longitud_genotipo);
//Crea valores aleatorios para los genotipos de los individuos de la poblacion
void crear_poblacion(poblacion *poblacion, int longitud_genotipo, double delta_t, double B, double limite_inferior, double limite_superior, double cord_obj_x, double cord_obj_y);
//Evalúa a la población
void evaluar_poblacion(poblacion *poblacion, int longitud_genotipo, double delta_t, double B, double cord_obj_x, double cord_obj_y);
//Evalúa a un individuo
double evaluar_individuo(double*u1, double *u2, int longitud_genotipo, double delta_t, double B, double cord_obj_x, double cord_obj_y);
//Ordena a la población de acuerdo a su fitness mediante el algoritmo de introsort
void ordenar_poblacion(poblacion *poblacion);
//Selecciona a los padres de la población mediante un torneo de fitness
void seleccionar_padres_torneo(poblacion *Poblacion, poblacion *padres, int num_competidores, int longitud_genotipo);
//Cruza a los padres para generar a los hijos dependiendo de una probabilidad de cruce
void cruzar_individuos(poblacion *padres, poblacion *hijos, int num_pob, int longitud_genotipo, double probabilidad_cruce, double delta_t, double B, double limite_inferior, double limite_superior, double nc, double cord_obj_x, double cord_obj_y);
//Muta a un individuo dependiendo de una probabilidad de mutación
void mutar_individuo(individuo *individuo, double probabilidad_mutacion, int longitud_genotipo, double limite_inferior, double limite_superior, double nm);
//Actualiza a la población con los nuevos individuos (hijos)
void actualizar_poblacion(poblacion **destino, poblacion *origen, int longitud_genotipo);
//Libera la memoria usada para la población
void liberar_poblacion(poblacion *poblacion);

//Funciones auxiliares del cruzamiento
// Función SBX para un solo gen (una variable)
// Recibe el valor del gen de cada padre, y genera dos hijos
void sbx_crossover(double parent1, double parent2, double *child1, double *child2,
    double nc, double limite_inferior, double limite_superior);

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

//Runge Kutta
void runge_kutta(double delta_t, double* condicion_inicial_x, double* condicion_inicial_y, double* condicion_inicial_phi, double* condicion_inicial_vl, double* condicion_inicial_vr, double* condicion_inicial_u1, double* condicion_inicial_u2, double B);
double x(double vl, double vr, double phi);
double y(double vl, double vr, double phi);
double phi(double vl, double vr, double B);

void imprimir_poblacion(poblacion *p, int longitud_genotipo);
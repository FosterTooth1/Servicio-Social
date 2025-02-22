#include "Biblioteca.h"

// Funciones principales del algoritmo genético

// Asigna memoria para una población
// Recibe el tamaño de la población y la longitud del genotipo
// Devuelve un puntero a la población creada
poblacion *inicializar_poblacion(int tamano, int longitud_genotipo)
{
    // Asigna memoria para la estructura de la población
    poblacion *Poblacion = malloc(sizeof(poblacion));

    // Asigna memoria para los individuos
    Poblacion->tamano = tamano;
    Poblacion->individuos = malloc(tamano * sizeof(individuo));

    // Asigna memoria para los genotipos de cada individuo
    for (int i = 0; i < tamano; i++)
    {
        Poblacion->individuos[i].genotipo_izquierdo = malloc(longitud_genotipo * sizeof(double));
        Poblacion->individuos[i].genotipo_derecho = malloc(longitud_genotipo * sizeof(double));

        // Inicializa el fitness en 0
        Poblacion->individuos[i].fitness = 0;
    }
    return Poblacion;
}


void crear_poblacion(poblacion *poblacion, int longitud_genotipo, double delta_t, double B)
{
    // Parámetros de la simulación y del controlador PID
    double kP = 0.2;       // ganancia proporcional
    double kI = 0.1;      // ganancia integral
    double kD = 0.1;      // ganancia derivativa

    // Condiciones iniciales para cada simulación individual
    double init_x = 0.0, init_y = 0.0, init_phi = 0.0;
    double init_vl = 0.0, init_vr = 0.0;
    // Definimos el objetivo (target) al que se desea llegar
    double target_x = 15.0, target_y = 0.0;

    // Para cada individuo de la población
    for (int i = 0; i < poblacion->tamano; i++)
    {
        // Se inicializan las variables de estado para cada individuo
        double x = init_x, y = init_y, phi_val = init_phi;
        double vl = init_vl, vr = init_vr;
        // Se generan aceleraciones iniciales aleatorias para cada llanta en el rango [-0.5, 0.5]
        double u1 = -0.5 + ((double)rand() / (double)RAND_MAX) * 1.0;
        //double u2 = -0.5 + ((double)rand() / (double)RAND_MAX) * 1.0;

        // Se genera una variación pequeña en el rango [-0.05, 0.05]
        double variacion = -0.05 + ((double)rand() / RAND_MAX) * 0.1;

        // Se obtiene u2 añadiéndole la variación a u1
        double u2 = u1 + variacion;

        // Se asegura que u2 se mantenga en el rango [-0.5, 0.5]
        if (u2 > 0.5) u2 = 0.5;
        if (u2 < -0.5) u2 = -0.5;

        // Guardamos las condiciones iniciales del genotipo
        poblacion->individuos[i].genotipo_izquierdo[0] = u1;
        poblacion->individuos[i].genotipo_derecho[0]   = u2;

        // Calculamos la distancia inicial al objetivo
        double prev_distance = sqrt((x - target_x) * (x - target_x) +
                                    (y - target_y) * (y - target_y));

        // Inicializamos variables para el controlador PID
        double integral_error = 0.0;
        double previous_error = 0.0;

        // Para cada instante de tiempo (a partir del segundo) se simula la acción del controlador
        for (int j = 1; j < longitud_genotipo; j++)
        {
            // Se actualiza el estado del robot (posición, orientación, velocidades y aceleraciones)
            runge_kutta(delta_t, &x, &y, &phi_val, &vl, &vr, &u1, &u2, B);

            // Se calcula la distancia actual al objetivo
            double current_distance = sqrt((x - target_x) * (x - target_x) +
                                           (y - target_y) * (y - target_y));

            // Si la distancia aumenta, se aplica la acción del controlador PID
            if (current_distance > prev_distance)
            {
                // Se calcula el ángulo deseado hacia el objetivo
                double desired_angle = atan2(target_y - y, target_x - x);
                // Se determina el error entre el ángulo deseado y la orientación actual
                double error_angle = desired_angle - phi_val;
                // Normalizamos error_angle al rango [-pi, pi]
                while (error_angle > M_PI)
                    error_angle -= 2.0 * M_PI;
                while (error_angle < -M_PI)
                    error_angle += 2.0 * M_PI;

                // Se acumula el error para el término integral
                integral_error += error_angle * delta_t;
                // Se calcula el término derivativo
                double derivative_error = (error_angle - previous_error) / delta_t;
                previous_error = error_angle;

                // Se calcula el ajuste basado en la suma de los términos PID
                double adjustment = kP * error_angle + kI * integral_error + kD * derivative_error;

                // Ajustamos las aceleraciones: se disminuye u1 e incrementa u2 (o viceversa)
                u1 -= adjustment;
                u2 += adjustment;

                // Se limitan las aceleraciones al rango permitido [-0.5, 0.5]
                if (u1 > 0.5) u1 = 0.5;
                if (u1 < -0.5) u1 = -0.5;
                if (u2 > 0.5) u2 = 0.5;
                if (u2 < -0.5) u2 = -0.5;
            }

            // Se guarda el par de aceleraciones calculado en el genotipo del individuo
            poblacion->individuos[i].genotipo_izquierdo[j] = u1;
            poblacion->individuos[i].genotipo_derecho[j]   = u2;

            // Se actualiza la distancia previa para la siguiente iteración
            prev_distance = current_distance;
        }
    }
}


// Evalua la población
// Recibe un puntero a la población y la longitud del genotipo
// No devuelve nada (todo se hace por referencia)
void evaluar_poblacion(poblacion *poblacion, int longitud_genotipo, double delta_t, double B)
{
    // Evalua cada individuo de la población
    for (int i = 0; i < poblacion->tamano; i++)
    {
        poblacion->individuos[i].fitness = evaluar_individuo(poblacion->individuos[i].genotipo_izquierdo, poblacion->individuos[i].genotipo_derecho, longitud_genotipo, delta_t, B);
    }
}

// Función para calcular la distancia total recorrida por el individuo (fitness)
// Recibe un genotipo y la longitud del genotipo
// Devuelve el fitness del individuo
double evaluar_individuo(double *u1, double *u2, int longitud_genotipo, double delta_t, double B)
{
    double total_cost = 0.0;
    double condicion_inicial_x = 0.0;
    double condicion_inicial_y = 0.0;
    double condicion_inicial_phi = 0.0;
    double condicion_inicial_vl = 0.0;
    double condicion_inicial_vr = 0.0;
    double condicion_inicial_u1;
    double condicion_inicial_u2;

    for (int i = 0; i < longitud_genotipo; i++)
    {
        condicion_inicial_u1 = u1[i];
        condicion_inicial_u2 = u2[i];
        runge_kutta(delta_t, &condicion_inicial_x, &condicion_inicial_y, &condicion_inicial_phi, &condicion_inicial_vl, &condicion_inicial_vr, &condicion_inicial_u1, &condicion_inicial_u2, B);
    }
    // Calculamos la distancia actual al punto objetivo
    total_cost = sqrt((condicion_inicial_x - 15) * (condicion_inicial_x - 15) + (condicion_inicial_y - 0) * (condicion_inicial_y - 0));

    return total_cost;
}

// Runge Kutta
void runge_kutta(double delta_t, double *condicion_inicial_x, double *condicion_inicial_y, double *condicion_inicial_phi, double *condicion_inicial_vl, double *condicion_inicial_vr, double *condicion_inicial_u1, double *condicion_inicial_u2, double B)
{

    // Calculo K1
    double k1_x = x(*condicion_inicial_vl, *condicion_inicial_vr, *condicion_inicial_phi);
    double k1_y = y(*condicion_inicial_vl, *condicion_inicial_vr, *condicion_inicial_phi);
    double k1_phi = phi(*condicion_inicial_vl, *condicion_inicial_vr, B);
    double k1_vl = *condicion_inicial_u1;
    double k1_vr = *condicion_inicial_u2;

    // Calculo K2
    double k2_x = x(*condicion_inicial_vl + (k1_vl) * ((0.5) * (delta_t)), *condicion_inicial_vr + (k1_vr) * ((0.5) * (delta_t)), *condicion_inicial_phi + (k1_phi) * ((0.5) * (delta_t)));
    double k2_y = y(*condicion_inicial_vl + (k1_vl) * ((0.5) * (delta_t)), *condicion_inicial_vr + (k1_vr) * ((0.5) * (delta_t)), *condicion_inicial_phi + (k1_phi) * ((0.5) * (delta_t)));
    double k2_phi = phi(*condicion_inicial_vl + (k1_vl) * ((0.5) * (delta_t)), *condicion_inicial_vr + (k1_vr) * ((0.5) * (delta_t)), B);
    double k2_vl = k1_vl;
    double k2_vr = k1_vr;

    // Calculo k3
    double k3_x = x(*condicion_inicial_vl + (k2_vl) * ((0.5) * (delta_t)), *condicion_inicial_vr + (k2_vr) * ((0.5) * (delta_t)), *condicion_inicial_phi + (k2_phi) * ((0.5) * (delta_t)));
    double k3_y = y(*condicion_inicial_vl + (k2_vl) * ((0.5) * (delta_t)), *condicion_inicial_vr + (k2_vr) * ((0.5) * (delta_t)), *condicion_inicial_phi + (k2_phi) * ((0.5) * (delta_t)));
    double k3_phi = phi(*condicion_inicial_vl + (k2_vl) * ((0.5) * (delta_t)), *condicion_inicial_vr + (k2_vr) * ((0.5) * (delta_t)), B);
    double k3_vl = k1_vl;
    double k3_vr = k1_vr;

    // Calculo k4
    double k4_x = x(*condicion_inicial_vl + (k3_vl) * (delta_t), *condicion_inicial_vr + (k3_vr) * (delta_t), *condicion_inicial_phi + (k3_phi) * (delta_t));
    double k4_y = y(*condicion_inicial_vl + (k3_vl) * (delta_t), *condicion_inicial_vr + (k3_vr) * (delta_t), *condicion_inicial_phi + (k3_phi) * (delta_t));
    double k4_phi = phi(*condicion_inicial_vl + (k3_vl) * (delta_t), *condicion_inicial_vr + (k3_vr) * (delta_t), B);
    double k4_vl = k1_vl;
    double k4_vr = k1_vr;

    double new_condicion_inicial_x = (*condicion_inicial_x) + (((delta_t) / (6)) * (k1_x + ((2) * (k2_x)) + ((2) * (k3_x)) + k4_x));
    double new_condicion_inicial_y = (*condicion_inicial_y) + (((delta_t) / (6)) * (k1_y + ((2) * (k2_y)) + ((2) * (k3_y)) + k4_y));
    double new_condicion_inicial_phi = (*condicion_inicial_phi) + (((delta_t) / (6)) * (k1_phi + ((2) * (k2_phi)) + ((2) * (k3_phi)) + k4_phi));
    double new_condicion_inicial_vl = (*condicion_inicial_vl) + (((delta_t) / (6)) * (k1_vl + ((2) * (k2_vl)) + ((2) * (k3_vl)) + k4_vl));
    double new_condicion_inicial_vr = (*condicion_inicial_vr) + (((delta_t) / (6)) * (k1_vr + ((2) * (k2_vr)) + ((2) * (k3_vr)) + k4_vr));

    *condicion_inicial_x = new_condicion_inicial_x;
    *condicion_inicial_y = new_condicion_inicial_y;
    *condicion_inicial_phi = new_condicion_inicial_phi;
    *condicion_inicial_vl = new_condicion_inicial_vl;
    *condicion_inicial_vr = new_condicion_inicial_vr;
}

double x(double vl, double vr, double phi)
{
    return (((vl + vr) / 2) * (cos(phi)));
}

double y(double vl, double vr, double phi)
{
    return (((vl + vr) / 2) * (sin(phi)));
}

double phi(double vl, double vr, double B)
{
    return (vr - vl) / B;
}

// Función principal de ordenamiento para la población
// Recibe un puntero a la población
// No devuelve nada (todo se hace por referencia)
void ordenar_poblacion(poblacion *poblacion)
{
    // Obtenemos el tamaño de la población
    int n = poblacion->tamano;

    // Si la población igual o menor a 1, no se hace nada
    if (n <= 1)
        return;

    // Calculamos la profundidad máxima de recursión
    int profundidad_max = 2 * log2_suelo(n);

    // Llamamos a la función auxiliar de ordenamiento introspectivo
    introsort_util(poblacion->individuos, &profundidad_max, 0, n);
}

// Selección de padres mediante un torneo de fitness
// Recibe un puntero a la población, un puntero a la población de padres, el número de competidores y la longitud del genotipo
// No devuelve nada (todo se hace por referencia)
void seleccionar_padres_torneo(poblacion *Poblacion, poblacion *padres, int num_competidores, int longitud_genotipo)
{
    // Inicializamos un array para los índices de los competidores
    int tamano_poblacion = Poblacion->tamano;
    int *indices_torneo = malloc(num_competidores * sizeof(int));

    // Realizamos un torneo para seleccionar a un padre
    for (int i = 0; i < tamano_poblacion; i++)
    {

        // Seleccionamos al azar los competidores del torneo
        for (int j = 0; j < num_competidores; j++)
        {
            indices_torneo[j] = rand() % tamano_poblacion;
        }

        // Encontramos el ganador del torneo evaluando su fitness
        int indice_ganador = indices_torneo[0];
        double mejor_fitness = Poblacion->individuos[indices_torneo[0]].fitness;
        for (int j = 1; j < num_competidores; j++)
        {
            int indice_actual = indices_torneo[j];
            double fitness_actual = Poblacion->individuos[indice_actual].fitness;

            if (fitness_actual < mejor_fitness)
            {
                mejor_fitness = fitness_actual;
                indice_ganador = indice_actual;
            }
        }

        // Copiamos el individuo ganador a la población de padres
        for (int j = 0; j < longitud_genotipo; j++)
        {
            padres->individuos[i].genotipo_izquierdo[j] = Poblacion->individuos[indice_ganador].genotipo_izquierdo[j];
            padres->individuos[i].genotipo_derecho[j] = Poblacion->individuos[indice_ganador].genotipo_derecho[j];
        }

        // Copiamos el fitness del ganador
        padres->individuos[i].fitness = Poblacion->individuos[indice_ganador].fitness;
    }

    // Liberamos la memoria usada para los índices
    free(indices_torneo);
}

// Función de cruzamiento con SBX y selección de los mejores dos entre padres e hijos
void cruzar_individuos(poblacion *padres, poblacion *hijos, int num_pob, int longitud_genotipo, double probabilidad_cruce, double delta_t, double B) {
    // Se asume que num_pob es par.
    for (int i = 0; i < num_pob; i += 2) {
        // Definimos una estructura temporal para almacenar u1, u2 y fitness
        typedef struct {
            double *u1;
            double *u2;
            double fitness;
        } individuo_temp;

        individuo_temp temp[4];
        // Asignar memoria para los arreglos de cada individuo temporal
        for (int k = 0; k < 4; k++) {
            temp[k].u1 = malloc(longitud_genotipo * sizeof(double));
            temp[k].u2 = malloc(longitud_genotipo * sizeof(double));
        }

        // Copiar los padres en temp[0] y temp[1]
        for (int j = 0; j < longitud_genotipo; j++) {
            temp[0].u1[j] = padres->individuos[i].genotipo_izquierdo[j];
            temp[0].u2[j] = padres->individuos[i].genotipo_derecho[j];
            temp[1].u1[j] = padres->individuos[i+1].genotipo_izquierdo[j];
            temp[1].u2[j] = padres->individuos[i+1].genotipo_derecho[j];
        }

        // Si se cumple la probabilidad de cruzamiento, generar hijos con SBX;
        // de lo contrario, copiar directamente los padres a los hijos temporales.
        if (((double)rand() / RAND_MAX) < probabilidad_cruce) {
            for (int j = 0; j < longitud_genotipo; j++) {
                sbx_crossover(padres->individuos[i].genotipo_izquierdo[j],
                              padres->individuos[i+1].genotipo_izquierdo[j],
                              &temp[2].u1[j],
                              &temp[3].u1[j],
                              15, -0.5, 0.5);
                sbx_crossover(padres->individuos[i].genotipo_derecho[j],
                              padres->individuos[i+1].genotipo_derecho[j],
                              &temp[2].u2[j],
                              &temp[3].u2[j],
                              15, -0.5, 0.5);
            }
        } else {
            for (int j = 0; j < longitud_genotipo; j++) {
                temp[2].u1[j] = padres->individuos[i].genotipo_izquierdo[j];
                temp[2].u2[j] = padres->individuos[i].genotipo_derecho[j];
                temp[3].u1[j] = padres->individuos[i+1].genotipo_izquierdo[j];
                temp[3].u2[j] = padres->individuos[i+1].genotipo_derecho[j];
            }
        }

        // Evaluar a cada uno de los 4 individuos temporales
        for (int k = 0; k < 4; k++) {
            temp[k].fitness = evaluar_individuo(temp[k].u1, temp[k].u2, longitud_genotipo, delta_t, B);
        }

        // Seleccionar los dos individuos con menor fitness (mejor aptitud)
        int best1 = 0, best2 = 1;
        if (temp[best2].fitness < temp[best1].fitness) {
            int aux = best1;
            best1 = best2;
            best2 = aux;
        }
        for (int k = 2; k < 4; k++) {
            if (temp[k].fitness < temp[best1].fitness) {
                best2 = best1;
                best1 = k;
            } else if (temp[k].fitness < temp[best2].fitness) {
                best2 = k;
            }
        }

        // Copiar los mejores dos individuos a la población de hijos
        for (int j = 0; j < longitud_genotipo; j++) {
            hijos->individuos[i].genotipo_izquierdo[j] = temp[best1].u1[j];
            hijos->individuos[i].genotipo_derecho[j]   = temp[best1].u2[j];
            hijos->individuos[i+1].genotipo_izquierdo[j] = temp[best2].u1[j];
            hijos->individuos[i+1].genotipo_derecho[j]   = temp[best2].u2[j];
        }
        hijos->individuos[i].fitness   = temp[best1].fitness;
        hijos->individuos[i+1].fitness = temp[best2].fitness;

        // Liberar memoria de los arreglos temporales
        for (int k = 0; k < 4; k++) {
            free(temp[k].u1);
            free(temp[k].u2);
        }
    }
}

// Función de mutación polinomial para un individuo
void mutar_individuo(individuo *ind, double probabilidad_mutacion, int longitud_genotipo)
{
    for (int j = 0; j < longitud_genotipo; j++)
    {
        // Mutación para el gen en genotipo_izquierdo (u1)
        if (((double)rand() / RAND_MAX) < probabilidad_mutacion)
        {
            double x = ind->genotipo_izquierdo[j];
            double xl = -0.5;
            double xu = 0.5;
            // Calcular los parámetros delta1 y delta2
            double delta1 = (x - xl) / (xu - xl);
            double delta2 = (xu - x) / (xu - xl);
            double r = ((double)rand() / RAND_MAX);
            double mut_pow = 1.0 / (20 + 1.0);
            double delta_q;
            if (r <= 0.5)
            {
                double xy = 1.0 - delta1;
                double val = 2.0 * r + (1.0 - 2.0 * r) * pow(xy, 20 + 1);
                delta_q = pow(val, mut_pow) - 1.0;
            }
            else
            {
                double xy = 1.0 - delta2;
                double val = 2.0 * (1.0 - r) + 2.0 * (r - 0.5) * pow(xy, 20 + 1);
                delta_q = 1.0 - pow(val, mut_pow);
            }
            // Actualizar el gen y asegurarse de que se encuentre dentro de los límites
            x = x + delta_q * (xu - xl);
            if (x < xl) x = xl;
            if (x > xu) x = xu;
            ind->genotipo_izquierdo[j] = x;
        }

        // Mutación para el gen en genotipo_derecho (u2)
        if (((double)rand() / RAND_MAX) < probabilidad_mutacion)
        {
            double x = ind->genotipo_derecho[j];
            double xl = -0.5;
            double xu = 0.5;
            double delta1 = (x - xl) / (xu - xl);
            double delta2 = (xu - x) / (xu - xl);
            double r = ((double)rand() / RAND_MAX);
            double mut_pow = 1.0 / (20 + 1.0);
            double delta_q;
            if (r <= 0.5)
            {
                double xy = 1.0 - delta1;
                double val = 2.0 * r + (1.0 - 2.0 * r) * pow(xy, 20 + 1);
                delta_q = pow(val, mut_pow) - 1.0;
            }
            else
            {
                double xy = 1.0 - delta2;
                double val = 2.0 * (1.0 - r) + 2.0 * (r - 0.5) * pow(xy, 20 + 1);
                delta_q = 1.0 - pow(val, mut_pow);
            }
            x = x + delta_q * (xu - xl);
            if (x < xl) x = xl;
            if (x > xu) x = xu;
            ind->genotipo_derecho[j] = x;
        }
    }
}

// Actualiza la población destino copiando los datos de la población origen
// Recibe un puntero a la población destino, un puntero a la población origen y la longitud del genotipo
// No devuelve nada (todo se hace por referencia)
void actualizar_poblacion(poblacion **destino, poblacion *origen, int longitud_genotipo)
{
    // Crea una población temporal nueva
    poblacion *nueva = inicializar_poblacion(origen->tamano, longitud_genotipo);

    // Copia los datos (ambos genotipos y fitness)
    for (int i = 0; i < origen->tamano; i++)
    {
        for (int j = 0; j < longitud_genotipo; j++)
        {
            nueva->individuos[i].genotipo_izquierdo[j] = origen->individuos[i].genotipo_izquierdo[j];
            nueva->individuos[i].genotipo_derecho[j]   = origen->individuos[i].genotipo_derecho[j];
        }
        nueva->individuos[i].fitness = origen->individuos[i].fitness;
    }

    // Libera la población antigua si existe
    if (*destino != NULL)
    {
        for (int i = 0; i < (*destino)->tamano; i++)
        {
            if ((*destino)->individuos[i].genotipo_izquierdo != NULL)
            {
                free((*destino)->individuos[i].genotipo_izquierdo);
            }
            if ((*destino)->individuos[i].genotipo_derecho != NULL)
            {
                free((*destino)->individuos[i].genotipo_derecho);
            }
        }
        if ((*destino)->individuos != NULL)
        {
            free((*destino)->individuos);
        }
        free(*destino);
    }

    // Asigna la nueva población al destino
    *destino = nueva;
}


// Libera la memoria usada por una población
// Recibe un puntero a la población
// No devuelve nada (todo se hace por referencia)
void liberar_poblacion(poblacion *pob)
{
    // Verifica si la población es nula
    if (pob == NULL)
        return;

    // Libera la memoria de los genotipos de cada individuo
    if (pob->individuos != NULL)
    {
        for (int i = 0; i < pob->tamano; i++)
        {
            if (pob->individuos[i].genotipo_izquierdo != NULL)
            {
                free(pob->individuos[i].genotipo_izquierdo);
                pob->individuos[i].genotipo_izquierdo = NULL;
            }
            if (pob->individuos[i].genotipo_derecho != NULL)
            {
                free(pob->individuos[i].genotipo_derecho);
                pob->individuos[i].genotipo_derecho = NULL;
            }
        }
        free(pob->individuos);
        pob->individuos = NULL;
    }

    // Libera la memoria de la población
    free(pob);
}



// Funciones auxiliares del cruzamiento

// Función SBX para un solo gen (una variable)
// Recibe el valor del gen de cada padre, y genera dos hijos
void sbx_crossover(double parent1, double parent2, double *child1, double *child2,
                   double eta, double lower_bound, double upper_bound)
{
    // Si los padres son casi iguales, se copia el valor sin cambio.
    if (fabs(parent1 - parent2) < 1e-14)
    {
        *child1 = parent1;
        *child2 = parent2;
        return;
    }

    // Se genera un número aleatorio en [0,1]
    double u = ((double)rand() / RAND_MAX);
    double beta;

    if (u <= 0.5)
        beta = pow(2 * u, 1.0 / (eta + 1));
    else
        beta = pow(1.0 / (2 * (1 - u)), 1.0 / (eta + 1));

    // Cálculo de los hijos
    *child1 = 0.5 * ((parent1 + parent2) - beta * (parent2 - parent1));
    *child2 = 0.5 * ((parent1 + parent2) + beta * (parent2 - parent1));

    // Se asegura que los hijos estén dentro de los límites
    if (*child1 < lower_bound)
        *child1 = lower_bound;
    if (*child1 > upper_bound)
        *child1 = upper_bound;
    if (*child2 < lower_bound)
        *child2 = lower_bound;
    if (*child2 > upper_bound)
        *child2 = upper_bound;
}

// Funciones auxiliares de ordenamiento

// Implementación de ordenamiento introspectivo
// Recibe un array de individuos, la profundidad máxima de recursión, el índice de inicio y fin
// No devuelve nada (todo se hace por referencia)
void introsort_util(individuo *arr, int *profundidad_max, int inicio, int fin)
{
    // Calculamos el tamaño de la partición
    int tamano = fin - inicio;

    // Si el tamaño de la partición es pequeño, usamos el ordenamiento por inserción
    if (tamano < 16)
    {
        insertion_sort(arr, inicio, fin - 1);
        return;
    }

    // Si la profundidad máxima es cero, cambiamos a heapsort (para evitar peor caso de quicksort)
    if (*profundidad_max == 0)
    {
        heapsort(arr + inicio, tamano);
        return;
    }

    // En caso contrario, usamos quicksort
    (*profundidad_max)--;
    int pivote = particion(arr, inicio, fin - 1);
    introsort_util(arr, profundidad_max, inicio, pivote);
    introsort_util(arr, profundidad_max, pivote + 1, fin);
}

// Función para calcular el logaritmo en base 2 de un número entero (parte entera)
// Recibe un número entero
// Devuelve el logaritmo en base 2 (parte entera)
int log2_suelo(int n)
{
    int log = 0;
    while (n > 1)
    {
        n >>= 1;
        log++;
    }
    return log;
}

// Partición de quicksort usando la mediana de tres como pivote
// Recibe un array de individuos, los índices bajo y alto
// Devuelve el índice del pivote
int particion(individuo *arr, int bajo, int alto)
{
    // Encontramos el índice del pivote usando la mediana de tres
    int medio = bajo + (alto - bajo) / 2;
    int indice_pivote = mediana_de_tres(arr, bajo, medio, alto);

    // Movemos el pivote seleccionado al final del rango para facilitar partición
    intercambiar_individuos(&arr[indice_pivote], &arr[alto]);

    // Guardamos el elemento del pivote para comparación
    individuo pivote = arr[alto];

    // i indica la última posición donde los elementos son menores o iguales al pivote
    int i = bajo - 1;

    // Recorremos el rango desde `bajo` hasta `alto - 1` (excluyendo el pivote)
    for (int j = bajo; j < alto; j++)
    {
        // Si el elemento actual es menor o igual al pivote
        if (arr[j].fitness <= pivote.fitness)
        {
            i++;                                       // Avanzamos `i` para marcar la posición de intercambio
            intercambiar_individuos(&arr[i], &arr[j]); // Intercambiamos el elemento menor al pivote
        }
    }

    // Finalmente, colocamos el pivote en su posición correcta
    intercambiar_individuos(&arr[i + 1], &arr[alto]);

    // Retornamos la posición del pivote
    return i + 1;
}

// Función para encontrar la mediana de tres elementos (usado en quicksort para mejorar el balanceo)
// Recibe un array de individuos y tres índices
// Devuelve el índice de la mediana
int mediana_de_tres(individuo *arr, int a, int b, int c)
{
    // Se realizan comparaciones lógicas para encontrar la mediana
    if (arr[a].fitness <= arr[b].fitness)
    {
        if (arr[b].fitness <= arr[c].fitness)
            return b;
        else if (arr[a].fitness <= arr[c].fitness)
            return c;
        else
            return a;
    }
    else
    {
        if (arr[a].fitness <= arr[c].fitness)
            return a;
        else if (arr[b].fitness <= arr[c].fitness)
            return c;
        else
            return b;
    }
}

// Función para intercambiar dos elementos
// Recibe dos punteros a individuos
// No devuelve nada (todo se hace por referencia)
void intercambiar_individuos(individuo *a, individuo *b)
{
    individuo temp = *a;
    *a = *b;
    *b = temp;
}

// Ordenamiento por inserción para arreglos pequeños
// Recibe un array de individuos, el índice izquierdo y derecho
// No devuelve nada (todo se hace por referencia)
void insertion_sort(individuo *arr, int izquierda, int derecha)
{
    // Recorremos el array de izquierda a derecha
    for (int i = izquierda + 1; i <= derecha; i++)
    {
        // Insertamos el elemento actual en la posición correcta
        individuo clave = arr[i];
        int j = i - 1;

        // Movemos los elementos mayores que la clave a una posición adelante
        while (j >= izquierda && arr[j].fitness > clave.fitness)
        {
            arr[j + 1] = arr[j];
            j--;
        }

        // Insertamos la clave en la posición correcta
        arr[j + 1] = clave;
    }
}

// Heapsort para ordenar a los individuos por fitness
// Recibe un array de individuos y el tamaño del array
// No devuelve nada (todo se hace por referencia)
void heapsort(individuo *arr, int n)
{
    // Construimos el montón (heapify)
    for (int i = n / 2 - 1; i >= 0; i--)
        heapify(arr, n, i);

    // Extraemos los elementos del montón uno por uno
    for (int i = n - 1; i > 0; i--)
    {
        individuo temp = arr[0];
        arr[0] = arr[i];
        arr[i] = temp;
        heapify(arr, i, 0);
    }
}

// Función auxiliar para heapsort
// Recibe un array de individuos, el tamaño del array y un índice
// No devuelve nada (todo se hace por referencia)
void heapify(individuo *arr, int n, int i)
{
    // Inicializamos el mayor como el indice actual
    int mayor = i;

    // Calculamos los indices de los hijos izquierdo y derecho
    int izquierda = 2 * i + 1;
    int derecha = 2 * i + 2;

    // Si el hijo izquierdo es mayor que el padre actualizamos el mayor
    if (izquierda < n && arr[izquierda].fitness > arr[mayor].fitness)
        mayor = izquierda;

    // Si el hijo derecho es mayor que el padre actualizamos el mayor
    if (derecha < n && arr[derecha].fitness > arr[mayor].fitness)
        mayor = derecha;

    // Si el mayor no es el padre, intercambiamos y aplicamos heapify al subárbol
    if (mayor != i)
    {
        individuo temp = arr[i];
        arr[i] = arr[mayor];
        arr[mayor] = temp;
        heapify(arr, n, mayor);
    }
}

void imprimir_poblacion(poblacion *p, int longitud_genotipo)
{
    for (int i = 0; i < p->tamano; i++)
    {
        printf("Individuo %d:\n", i + 1);
        
        // Imprimir genotipo izquierdo
        printf("Genotipo izquierdo: ");
        for (int j = 0; j < longitud_genotipo; j++)
        {
            printf("%f ", p->individuos[i].genotipo_izquierdo[j]);
        }
        printf("\n");
        
        // Imprimir genotipo derecho
        printf("Genotipo derecho: ");
        for (int j = 0; j < longitud_genotipo; j++)
        {
            printf("%f ", p->individuos[i].genotipo_derecho[j]);
        }
        printf("\n");
        
        // Imprimir fitness
        printf("Fitness: %f\n", p->individuos[i].fitness);
        printf("-------------------------------\n");
    }
}

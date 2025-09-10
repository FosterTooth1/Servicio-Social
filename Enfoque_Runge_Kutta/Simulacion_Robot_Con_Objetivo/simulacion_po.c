#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

// Estructura para guardar el resultado de la simulación.
// Cada fila del arreglo contendrá 8 floats: [t, x, y, phi, vl, vr, u1, u2]
typedef struct
{
    int n;       // Número de iteraciones (filas)
    float *data; // Arreglo plano de datos (n * 8 elementos)
} SimulationResult;

// Prototipos de funciones auxiliares
float f_x(float vl, float vr, float phi);
float f_y(float vl, float vr, float phi);
float f_phi(float vl, float vr, float B);
void runge_kutta(float delta_t, float *cond_x, float *cond_y, float *cond_phi,
                 float *cond_vl, float *cond_vr, float *cond_u1, float *cond_u2, float B);

#ifdef __cplusplus
extern "C"
{
#endif

    // Función principal que se exporta para ejecutar la simulación.
    // Parámetros:
    //   delta_t: paso de tiempo
    //   limite_tiempo: tiempo máximo de simulación
    //   cond_x, cond_y, cond_phi: condiciones iniciales de posición y orientación
    //   cond_vl, cond_vr: condiciones iniciales de velocidad para la llanta izquierda y derecha
    //   B: parámetro para el cálculo de phi
    //   target_x, target_y: coordenadas del punto objetivo
    //
    // En cada iteración se calcula la distancia al punto objetivo y se ajustan las aceleraciones
    // si la distancia aumenta, haciendo que el robot gire en dirección al objetivo.
    EXPORT SimulationResult *simular(float delta_t, float limite_tiempo,
                                     float cond_x, float cond_y, float cond_phi,
                                     float cond_vl, float cond_vr, float B,
                                     float target_x, float target_y)
    {
        int n_iter = (int)(limite_tiempo / delta_t) + 1;
        SimulationResult *res = (SimulationResult *)malloc(sizeof(SimulationResult));
        if (!res)
            return NULL;
        res->n = n_iter;
        res->data = (float *)malloc(n_iter * 8 * sizeof(float));
        if (!res->data)
        {
            free(res);
            return NULL;
        }

        float t = 0.0f;
        float x = cond_x;
        float y = cond_y;
        float phi_val = cond_phi; // Para diferenciar de la función helper
        float vl = cond_vl;
        float vr = cond_vr;
        float u1, u2;

        // Inicializamos la semilla para números aleatorios
        srand((unsigned int)time(NULL));

        // Inicializamos las aceleraciones de las llantas aleatoriamente en el intervalo [-0.5, 0.5]
        u1 = ((float)rand() / RAND_MAX) - 0.5f;
        // Se genera una variación pequeña en el rango [-0.05, 0.05]
        float variacion = -0.05 + ((float)rand() / RAND_MAX) * 0.1;

        // Se obtiene u2 añadiéndole la variación a u1
        u2 = u1 + variacion;

        // Calculamos la distancia inicial al objetivo
        float prev_distance = sqrtf((x - target_x) * (x - target_x) + (y - target_y) * (y - target_y));

        int idx = 0;
        for (int i = 0; i < n_iter; i++)
        {
            // Guardamos la información de la iteración actual: [t, x, y, phi, vl, vr, u1, u2]
            res->data[idx++] = t;
            res->data[idx++] = x;
            res->data[idx++] = y;
            res->data[idx++] = phi_val;
            res->data[idx++] = vl;
            res->data[idx++] = vr;
            res->data[idx++] = u1;
            res->data[idx++] = u2;

            // Actualizamos el estado usando el método de Runge–Kutta
            runge_kutta(delta_t, &x, &y, &phi_val, &vl, &vr, &u1, &u2, B);
            t += delta_t;

            // Calculamos la distancia actual al punto objetivo
            float current_distance = sqrtf((x - target_x) * (x - target_x) + (y - target_y) * (y - target_y));

            if (current_distance > prev_distance)
            {

                // Calculamos el ángulo deseado hacia el objetivo
                float desired_angle = atan2f(target_y - y, target_x - x);
                // Calculamos el error de orientación
                float error_angle = desired_angle - phi_val;
                // Normalizamos error_angle a [-pi, pi]
                while (error_angle > M_PI)
                    error_angle -= 2.0f * M_PI;
                while (error_angle < -M_PI)
                    error_angle += 2.0f * M_PI;

                // Factor de corrección (ganancia)
                float k = 0.2f;
                float adjustment = k * error_angle;

                // Ajustamos las aceleraciones: se disminuye u1 e incrementa u2 (o viceversa)
                u1 -= adjustment;
                u2 += adjustment;

                // Limitamos las aceleraciones al rango [-0.5, 0.5]
                if (u1 > 0.5f)
                    u1 = 0.5f;
                if (u1 < -0.5f)
                    u1 = -0.5f;
                if (u2 > 0.5f)
                    u2 = 0.5f;
                if (u2 < -0.5f)
                    u2 = -0.5f;
            }

            // Actualizamos la distancia previa para la siguiente iteración
            prev_distance = current_distance;
        }
        return res;
    }

    // Función para liberar la memoria reservada para el resultado de la simulación.
    EXPORT void liberar_resultado(SimulationResult *res)
    {
        if (res)
        {
            if (res->data)
                free(res->data);
            free(res);
        }
    }

#ifdef __cplusplus
}
#endif

// Implementación de Runge–Kutta y funciones helper

void runge_kutta(float delta_t, float *cond_x, float *cond_y, float *cond_phi,
                 float *cond_vl, float *cond_vr, float *cond_u1, float *cond_u2, float B)
{
    // Cálculo de K1
    float k1_x = f_x(*cond_vl, *cond_vr, *cond_phi);
    float k1_y = f_y(*cond_vl, *cond_vr, *cond_phi);
    float k1_phi = f_phi(*cond_vl, *cond_vr, B);
    float k1_vl = *cond_u1;
    float k1_vr = *cond_u2;

    // Cálculo de K2
    float k2_x = f_x(*cond_vl + k1_vl * (0.5f * delta_t),
                     *cond_vr + k1_vr * (0.5f * delta_t),
                     *cond_phi + k1_phi * (0.5f * delta_t));
    float k2_y = f_y(*cond_vl + k1_vl * (0.5f * delta_t),
                     *cond_vr + k1_vr * (0.5f * delta_t),
                     *cond_phi + k1_phi * (0.5f * delta_t));
    float k2_phi = f_phi(*cond_vl + k1_vl * (0.5f * delta_t),
                         *cond_vr + k1_vr * (0.5f * delta_t), B);
    float k2_vl = k1_vl;
    float k2_vr = k1_vr;

    // Cálculo de K3
    float k3_x = f_x(*cond_vl + k2_vl * (0.5f * delta_t),
                     *cond_vr + k2_vr * (0.5f * delta_t),
                     *cond_phi + k2_phi * (0.5f * delta_t));
    float k3_y = f_y(*cond_vl + k2_vl * (0.5f * delta_t),
                     *cond_vr + k2_vr * (0.5f * delta_t),
                     *cond_phi + k2_phi * (0.5f * delta_t));
    float k3_phi = f_phi(*cond_vl + k2_vl * (0.5f * delta_t),
                         *cond_vr + k2_vr * (0.5f * delta_t), B);
    float k3_vl = k1_vl;
    float k3_vr = k1_vr;

    // Cálculo de K4
    float k4_x = f_x(*cond_vl + k3_vl * delta_t,
                     *cond_vr + k3_vr * delta_t,
                     *cond_phi + k3_phi * delta_t);
    float k4_y = f_y(*cond_vl + k3_vl * delta_t,
                     *cond_vr + k3_vr * delta_t,
                     *cond_phi + k3_phi * delta_t);
    float k4_phi = f_phi(*cond_vl + k3_vl * delta_t,
                         *cond_vr + k3_vr * delta_t, B);
    float k4_vl = k1_vl;
    float k4_vr = k1_vr;

    float new_x = (*cond_x) + (delta_t / 6.0f) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);
    float new_y = (*cond_y) + (delta_t / 6.0f) * (k1_y + 2 * k2_y + 2 * k3_y + k4_y);
    float new_phi = (*cond_phi) + (delta_t / 6.0f) * (k1_phi + 2 * k2_phi + 2 * k3_phi + k4_phi);
    float new_vl = (*cond_vl) + (delta_t / 6.0f) * (k1_vl + 2 * k2_vl + 2 * k3_vl + k4_vl);
    float new_vr = (*cond_vr) + (delta_t / 6.0f) * (k1_vr + 2 * k2_vr + 2 * k3_vr + k4_vr);

    *cond_x = new_x;
    *cond_y = new_y;
    *cond_phi = new_phi;
    *cond_vl = new_vl;
    *cond_vr = new_vr;
}

float f_x(float vl, float vr, float phi)
{
    return ((vl + vr) / 2.0f) * cosf(phi);
}

float f_y(float vl, float vr, float phi)
{
    return ((vl + vr) / 2.0f) * sinf(phi);
}

float f_phi(float vl, float vr, float B)
{
    return (vr - vl) / B;
}

#include<stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

void runge_kutta(float delta_t, float* condicion_inicial_x, float* condicion_inicial_y, float* condicion_inicial_phi, float* condicion_inicial_vl, float* condicion_inicial_vr, float* condicion_inicial_u1, float* condicion_inicial_u2, float B);
float x(float vl, float vr, float phi);
float y(float vl, float vr, float phi);
float phi(float vl, float vr, float B);


int main(){
    float delta_t = 0.01; // Paso de tiempo
    float valor_t = 0; // Tiempo inicial
    float limite_tiempo = 0.2; // Tiempo final
    float condicion_inicial_x = 0; // Posicion inicial en x
    float condicion_inicial_y = 0; // Posicion inicial en y
    float condicion_inicial_phi = 0; // Angulo inicial
    float condicion_inicial_vl = 0; // Velocidad inicial izquierda
    float condicion_inicial_vr = 0; // Velocidad inicial derecha
    float B = 0.20; // Distancia entre llantas
    float condicion_inicial_u1; // Izquierda
    float condicion_inicial_u2; // Derecha

    // Inicializamos la semilla para los números aleatorios
    srand((unsigned int)time(NULL));
    
    while(valor_t <=limite_tiempo){
        // Generamos valores aleatorios entre -0.5 y 0.5 en cada iteracion para la aceleracion de cada llanta
        condicion_inicial_u1 = ((float)rand() / RAND_MAX) - 0.5;
        condicion_inicial_u2 = ((float)rand() / RAND_MAX) - 0.5;

        printf("t = %f, x = %f, y = %f, phi = %f, vl = %f, vr = %f, u1 = %f, u2 = %f \n", valor_t , condicion_inicial_x, condicion_inicial_y, condicion_inicial_phi, condicion_inicial_vl, condicion_inicial_vr, condicion_inicial_u1, condicion_inicial_u2);
        runge_kutta(delta_t, &condicion_inicial_x, &condicion_inicial_y, &condicion_inicial_phi, &condicion_inicial_vl, &condicion_inicial_vr, &condicion_inicial_u1, &condicion_inicial_u2, B);
        valor_t = valor_t  + delta_t;
    }

}

void runge_kutta(float delta_t, float* condicion_inicial_x, float* condicion_inicial_y, float* condicion_inicial_phi, float* condicion_inicial_vl, float* condicion_inicial_vr, float* condicion_inicial_u1, float* condicion_inicial_u2, float B){

    // Calculo K1
    float k1_x = x(*condicion_inicial_vl, *condicion_inicial_vr, *condicion_inicial_phi);
    float k1_y = y(*condicion_inicial_vl, *condicion_inicial_vr, *condicion_inicial_phi);
    float k1_phi = phi(*condicion_inicial_vl, *condicion_inicial_vr, B);
    float k1_vl = *condicion_inicial_u1;
    float k1_vr = *condicion_inicial_u2;

    // Calculo K2
    float k2_x = x(*condicion_inicial_vl + (k1_vl)*((0.5)*(delta_t)), *condicion_inicial_vr + (k1_vr)*((0.5)*(delta_t)), *condicion_inicial_phi + (k1_phi)*((0.5)*(delta_t)));
    float k2_y = y(*condicion_inicial_vl + (k1_vl)*((0.5)*(delta_t)), *condicion_inicial_vr + (k1_vr)*((0.5)*(delta_t)), *condicion_inicial_phi + (k1_phi)*((0.5)*(delta_t)));
    float k2_phi = phi(*condicion_inicial_vl + (k1_vl)*((0.5)*(delta_t)), *condicion_inicial_vr + (k1_vr)*((0.5)*(delta_t)), B);
    float k2_vl = k1_vl;
    float k2_vr = k1_vr;

    // Calculo k3
    float k3_x = x(*condicion_inicial_vl + (k2_vl)*((0.5)*(delta_t)), *condicion_inicial_vr + (k2_vr)*((0.5)*(delta_t)), *condicion_inicial_phi + (k2_phi)*((0.5)*(delta_t)));
    float k3_y = y(*condicion_inicial_vl + (k2_vl)*((0.5)*(delta_t)), *condicion_inicial_vr + (k2_vr)*((0.5)*(delta_t)), *condicion_inicial_phi + (k2_phi)*((0.5)*(delta_t)));
    float k3_phi = phi(*condicion_inicial_vl + (k2_vl)*((0.5)*(delta_t)), *condicion_inicial_vr + (k2_vr)*((0.5)*(delta_t)), B);
    float k3_vl = k1_vl;
    float k3_vr = k1_vr;

    // Calculo k4
    float k4_x = x(*condicion_inicial_vl + (k3_vl)*(delta_t), *condicion_inicial_vr + (k3_vr)*(delta_t), *condicion_inicial_phi + (k3_phi)*(delta_t));
    float k4_y = y(*condicion_inicial_vl + (k3_vl)*(delta_t), *condicion_inicial_vr + (k3_vr)*(delta_t), *condicion_inicial_phi + (k3_phi)*(delta_t));
    float k4_phi = phi(*condicion_inicial_vl + (k3_vl)*(delta_t), *condicion_inicial_vr + (k3_vr)*(delta_t), B);
    float k4_vl = k1_vl;
    float k4_vr = k1_vr;



    float new_condicion_inicial_x = (*condicion_inicial_x) + (((delta_t)/(6)) * (k1_x + ((2)*(k2_x)) + ((2)*(k3_x)) + k4_x));
    float new_condicion_inicial_y = (*condicion_inicial_y) + (((delta_t)/(6)) * (k1_y + ((2)*(k2_y)) + ((2)*(k3_y)) + k4_y));
    float new_condicion_inicial_phi = (*condicion_inicial_phi) + (((delta_t)/(6)) * (k1_phi + ((2)*(k2_phi)) + ((2)*(k3_phi)) + k4_phi));
    float new_condicion_inicial_vl = (*condicion_inicial_vl) + (((delta_t)/(6)) * (k1_vl + ((2)*(k2_vl)) + ((2)*(k3_vl)) + k4_vl));
    float new_condicion_inicial_vr = (*condicion_inicial_vr) + (((delta_t)/(6)) * (k1_vr + ((2)*(k2_vr)) + ((2)*(k3_vr)) + k4_vr));



    *condicion_inicial_x = new_condicion_inicial_x;
    *condicion_inicial_y = new_condicion_inicial_y;
    *condicion_inicial_phi = new_condicion_inicial_phi;
    *condicion_inicial_vl = new_condicion_inicial_vl;
    *condicion_inicial_vr = new_condicion_inicial_vr;
}

float x(float vl, float vr, float phi){
    return (((vl + vr)/2)*(cos(phi)));
}

float y(float vl, float vr, float phi){
    return (((vl + vr)/2)*(sin(phi)));
}

float phi(float vl, float vr, float B){
    return (vr - vl)/B;
}
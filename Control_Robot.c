#include<stdio.h>

void runge_kutta(float delta_t, float* condicion_inicial_x, float* condicion_inicial_y, float* condicion_inicial_phi, float* condicion_inicial_vl, float* condicion_inicial_vr, float valor_t);
float x(float vl, float vr, float phi);
float y(float vl, float vr, float phi);
float phi(float vl, float vr);
float vl(float u1);
float vr(float u2);


int main(){
    float delta_t = 0.01;
    float valor_t = 0;
    float limite_tiempo = 4;
    float condicion_inicial_x = 0;
    float condicion_inicial_y = 0;
    float condicion_inicial_phi = 0;
    float condicion_inicial_vl = 0;
    float condicion_inicial_vr = 0;
    float u1;
    float u2;
    
    while(valor_t <=limite_tiempo){
        printf("t = %f, x = %f, y = %f, phi = %f, vl = %f, vr = %f, u1 = %f, u2 = %f \n", valor_t , condicion_inicial_x, condicion_inicial_y, condicion_inicial_phi, condicion_inicial_vl, condicion_inicial_vr, u1, u2);
        runge_kutta(delta_t, &condicion_inicial_x, &condicion_inicial_y, &condicion_inicial_phi, &condicion_inicial_vl, &condicion_inicial_vr, valor_t);
        valor_t = valor_t  + delta_t;
    }

}

void runge_kutta(float delta_t, float* condicion_inicial_x, float* condicion_inicial_y, float* condicion_inicial_phi, float* condicion_inicial_vl, float* condicion_inicial_vr, float valor_t){
    float k1_f1 = f1(*condicion_inicial_y1);
    float k2_f1 = f1(*condicion_inicial_y1 + (k1_f1)*((0.5)*(delta_t)));
    float k3_f1 = f1(*condicion_inicial_y1 + (k2_f1)*((0.5)*(delta_t)));
    float k4_f1 = f1(*condicion_inicial_y1 + (k3_f1)*(delta_t));

    float k1_f2 = f2(*condicion_inicial_y1, *condicion_inicial_y2);
    float k2_f2 = f2(*condicion_inicial_y1 + (k1_f1)*((0.5)*(delta_t)), *condicion_inicial_y2 + (k1_f2)*((0.5)*(delta_t)));
    float k3_f2 = f2(*condicion_inicial_y1 + (k2_f1)*((0.5)*(delta_t)), *condicion_inicial_y2 + (k2_f2)*((0.5)*(delta_t)));
    float k4_f2 = f2(*condicion_inicial_y1 + (k3_f1)*(delta_t), *condicion_inicial_y2 + (k3_f2)*(delta_t));

    float new_condicion_inicial_y1 = (*condicion_inicial_y1) + (((delta_t)/(6)) * (k1_f1 + ((2)*(k2_f1)) + ((2)*(k3_f1)) + k4_f1));
    float new_condicion_inicial_y2 = (*condicion_inicial_y2) + (((delta_t)/(6)) * (k1_f2 + ((2)*(k2_f2)) + ((2)*(k3_f2)) + k4_f2));

    *condicion_inicial_y1 = new_condicion_inicial_y1;
    *condicion_inicial_y2 = new_condicion_inicial_y2;
}

float f1(float y_1){
    return ((-0.5)*(y_1));
}

float f2(float y_1, float y_2){
    return (4 -(0.3)*(y_2) - (0.1)*(y_1));
}
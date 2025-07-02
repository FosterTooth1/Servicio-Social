#include<stdio.h>

void runge_kutta(float delta_x, float* condicion_inicial, float valor_x);
float f(float x);

int main(){
    float delta_x = 0.25;
    float valor_x = 0;
    float limite_superior = 4;
    float condicion_inicial = 1;
    
    while(valor_x <=limite_superior){
        printf("x = %f, y = %f \n", valor_x , condicion_inicial);
        runge_kutta(delta_x, &condicion_inicial, valor_x);
        valor_x = valor_x  + delta_x;
    }

}

void runge_kutta(float delta_x, float* condicion_inicial, float valor_x){
    float k1 = f(valor_x);
    float k2 = f(valor_x + ((0.5)*(delta_x)));
    float k3 = f(valor_x + ((0.5)*(delta_x)));
    float k4 = f(valor_x + delta_x);
    float new_condicion_inicial = (*condicion_inicial) + (((delta_x)/(6)) * (k1 + ((2)*(k2)) + ((2)*(k3)) + k4));
    *condicion_inicial = new_condicion_inicial;
}

float f(float x){
    return (-2*x*x*x) + (12*x*x) - (20*x) + 8.5;
}
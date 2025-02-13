#include<stdio.h>

void runge_kutta(float delta_x, float* condicion_inicial, float valor_x);
float f(float x);

int main(){
    float delta_x = 0.5;
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
    float new_condicion_inicial = (*condicion_inicial) + (delta_x)*(k1);
    *condicion_inicial = new_condicion_inicial;
}

float f(float x){
    return (-2*x*x*x) + (12*x*x) - (20*x) + 8.5;
}

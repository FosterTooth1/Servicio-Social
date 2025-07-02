#include<stdio.h>

void runge_kutta(float delta_x, float* condicion_inicial_y1, float* condicion_inicial_y2,float valor_x);
float f1(float y_1);
float f2(float y_1, float y_2);

int main(){
    float delta_x = 0.25;
    float valor_x = 0;
    float limite_superior = 4;
    float condicion_inicial_y1 = 4;
    float condicion_inicial_y2 = 6;
    
    while(valor_x <=limite_superior){
        printf("x = %f, y1 = %f, y2 = %f \n", valor_x , condicion_inicial_y1, condicion_inicial_y2);
        runge_kutta(delta_x, &condicion_inicial_y1, &condicion_inicial_y2, valor_x);
        valor_x = valor_x  + delta_x;
    }

}

void runge_kutta(float delta_x, float* condicion_inicial_y1, float* condicion_inicial_y2, float valor_x){
    float k1_f1 = f1(*condicion_inicial_y1);
    float k2_f1 = f1(*condicion_inicial_y1 + (k1_f1)*((0.5)*(delta_x)));
    float k3_f1 = f1(*condicion_inicial_y1 + (k2_f1)*((0.5)*(delta_x)));
    float k4_f1 = f1(*condicion_inicial_y1 + (k3_f1)*(delta_x));

    float k1_f2 = f2(*condicion_inicial_y1, *condicion_inicial_y2);
    float k2_f2 = f2(*condicion_inicial_y1 + (k1_f1)*((0.5)*(delta_x)), *condicion_inicial_y2 + (k1_f2)*((0.5)*(delta_x)));
    float k3_f2 = f2(*condicion_inicial_y1 + (k2_f1)*((0.5)*(delta_x)), *condicion_inicial_y2 + (k2_f2)*((0.5)*(delta_x)));
    float k4_f2 = f2(*condicion_inicial_y1 + (k3_f1)*(delta_x), *condicion_inicial_y2 + (k3_f2)*(delta_x));

    float new_condicion_inicial_y1 = (*condicion_inicial_y1) + (((delta_x)/(6)) * (k1_f1 + ((2)*(k2_f1)) + ((2)*(k3_f1)) + k4_f1));
    float new_condicion_inicial_y2 = (*condicion_inicial_y2) + (((delta_x)/(6)) * (k1_f2 + ((2)*(k2_f2)) + ((2)*(k3_f2)) + k4_f2));

    *condicion_inicial_y1 = new_condicion_inicial_y1;
    *condicion_inicial_y2 = new_condicion_inicial_y2;
}

float f1(float y_1){
    return ((-0.5)*(y_1));
}

float f2(float y_1, float y_2){
    return (4 -(0.3)*(y_2) - (0.1)*(y_1));
}
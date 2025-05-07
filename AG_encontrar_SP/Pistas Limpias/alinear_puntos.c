#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//#define M_PI 3.14159265

typedef struct {
    double x;
    double y;
} Punto;    

typedef struct {
    Punto *puntos;
    int num_puntos;
} Circuito;

void leer_puntos(Circuito *Circuito);
void imprimir_puntos(Circuito *Circuito);
void liberar_puntos(Circuito *Circuito);
double calcular_angulo_tres_puntos(Punto p1, Punto p2, Punto p3);
void rectificar_circuito(Circuito *Circuito);
void calcular_distancia(Circuito *Circuito);

/* Esta funcion lee los puntos de un circuito en un archivo csv 
y los almacena en una estructura de datos. El archivo csv debe tener el siguiente formato:
x,y
3.846912496816170233e+02,-1.296688540366591837e+02
5.878492682713128943e+02,-1.590923829351150118e+02
*/
void leer_puntos(Circuito *Circuito) {
    FILE *archivo = fopen("pista_escalada.csv", "r");
    if (archivo == NULL) {
        perror("Error al abrir el archivo");
        exit(EXIT_FAILURE);
    }

    // Leer y descartar el encabezado "x,y"
    char encabezado[256];
    if (fgets(encabezado, sizeof(encabezado), archivo) == NULL) {
        fclose(archivo);
        fprintf(stderr, "Error: Archivo vacío o sin encabezado\n");
        exit(EXIT_FAILURE);
    }

    Circuito->num_puntos = 0;
    Circuito->puntos = NULL;

    char linea[256];
    while (fgets(linea, sizeof(linea), archivo)) {
        // Redimensionar el array de puntos de forma segura
        Punto *temp = realloc(Circuito->puntos, (Circuito->num_puntos + 1) * sizeof(Punto));
        if (temp == NULL) {
            perror("Error al asignar memoria");
            fclose(archivo);
            free(Circuito->puntos); // Liberar memoria existente en caso de error
            exit(EXIT_FAILURE);
        }
        Circuito->puntos = temp;
        
        // Leer coordenadas x e y como doubles
        int campos_leidos = sscanf(
            linea, 
            "%lf,%lf", 
            &Circuito->puntos[Circuito->num_puntos].x, 
            &Circuito->puntos[Circuito->num_puntos].y
        );
        
        if (campos_leidos != 2) {
            fprintf(stderr, "Error en el formato de la línea: %s", linea);
            fclose(archivo);
            free(Circuito->puntos);
            exit(EXIT_FAILURE);
        }
        
        Circuito->num_puntos++;
    }

    fclose(archivo);
}

void imprimir_puntos(Circuito *Circuito) {
    for (int i = 0; i < Circuito->num_puntos; i++) {
        printf("Punto %d: (%.2f, %.2f)\n", i + 1, Circuito->puntos[i].x, Circuito->puntos[i].y);
    }
}

double calcular_angulo_tres_puntos(Punto p1, Punto p2, Punto p3) {
    // Vectores DESDE p1 HACIA p2 y p3
    double vec1_x = p2.x - p1.x;
    double vec1_y = p2.y - p1.y;
    double vec2_x = p3.x - p1.x;
    double vec2_y = p3.y - p1.y;
    
    // Producto punto
    double dot = vec1_x * vec2_x + vec1_y * vec2_y;
    
    // Magnitudes
    double mag1 = sqrt(vec1_x * vec1_x + vec1_y * vec1_y);
    double mag2 = sqrt(vec2_x * vec2_x + vec2_y * vec2_y);
    
    if (mag1 == 0 || mag2 == 0) return 0.0;
    
    // Coseno del ángulo
    double cos_theta = dot / (mag1 * mag2);
    cos_theta = fmax(fmin(cos_theta, 1.0), -1.0);  // Forzar rango [-1, 1]
    
    return acos(cos_theta) * (180.0 / M_PI);  // Convertir a grados
}

void rectificar_circuito(Circuito *Circuito) {
    // Necesitamos al menos 3 puntos para formar un ángulo
    if (Circuito->num_puntos < 3) return;
    
    for (int i = 0; i < Circuito->num_puntos - 2; ++i) {
        Punto p1 = Circuito->puntos[i];
        Punto p2 = Circuito->puntos[i + 1];
        Punto p3 = Circuito->puntos[i + 2];
        
        double angulo = calcular_angulo_tres_puntos(p1, p2, p3);
        printf("Ángulo en trío %d-%d-%d: %.2f grados\n", 
               i + 1, i + 2, i + 3, angulo);
    }
}

int main() {
    Circuito circuito;
    leer_puntos(&circuito);
    rectificar_circuito(&circuito);
    imprimir_puntos(&circuito);
    //liberar_puntos(&circuito);
    return 0;
}   
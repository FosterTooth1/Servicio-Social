import numpy as np
import matplotlib.pyplot as plt

# Solicitar parámetros al usuario
num_iteraciones = int(input('Ingrese el número de veces que se repetirá el algoritmo: '))
num_generaciones = int(input('Ingrese el número de generaciones en el algoritmo: '))
num_pob = int(input('Ingrese el numero de individuos dentro de la población: '))
num_var = int(input('Ingrese la cantidad de variables que tiene cada individuo: '))

limite_inferior = np.zeros(num_var)
for i in range(num_var):
    limite_inferior[i] = float(input(f'Ingrese el limite inferior de la variable {i+1}: '))

limite_superior = np.zeros(num_var)
for i in range(num_var):
    limite_superior[i] = float(input(f'Ingrese el limite superior de la variable {i+1}: '))

alpha_sharing = float(input('Ingrese el valor de alpha en el fitness sharing: '))
sigma_share = float(input('Ingrese el valor del radio de los nichos: '))
Pc = float(input('Ingrese la probabilidad de cruza del algoritmo: '))
Pm = float(input('Ingrese la probabilidad de mutacion del algoritmo: '))

lambda_P = 1000

# Inicializar población
poblacion = np.random.uniform(
    low=limite_inferior,
    high=limite_superior,
    size=(num_pob, num_var)
)

resultados_generales = np.zeros(num_iteraciones)
mejor_individuo_general = np.zeros((num_iteraciones, num_var))

for iteracion in range(num_iteraciones):
    mejor_aptitud = np.zeros(num_generaciones)
    mejor_individuo = np.zeros((num_generaciones, num_var))
    
    fig = plt.figure(iteracion)
    
    for generacion in range(num_generaciones):
        # Determinar parámetros evolutivos
        if generacion < num_generaciones * 0.5:
            Nc, Nm = 2, 20
        elif generacion < num_generaciones * 0.75:
            Nc, Nm = 5, 50
        elif generacion < num_generaciones * 0.80:
            Nc, Nm = 10, 75
        elif generacion < num_generaciones * 0.95:
            Nc, Nm = 15, 85
        else:
            Nc, Nm = 20, 100

        # Evaluar aptitud
        aptitud = np.zeros(num_pob)
        for i in range(num_pob):
            x1, x2 = poblacion[i, 0], poblacion[i, 1]
            f = 4*(x1 - 3)**2 + 3*(x2 - 3)**2
            
            # Restricciones
            g1 = 2*x1 + x2 - 2
            g2 = 3*x1 + 4*x2 - 6
            penalizacion = sum(np.maximum([g1, g2], 0)**2)
            
            aptitud[i] = f + lambda_P * penalizacion

        # Fitness sharing
        aptitud_compartida = np.copy(aptitud)
        for i in range(num_pob):
            compartido = 0
            for j in range(num_pob):
                if i != j:
                    distancia = np.linalg.norm(poblacion[i] - poblacion[j])
                    if distancia < sigma_share:
                        sh = 1 - (distancia/sigma_share)**alpha_sharing
                        compartido += sh
            aptitud_compartida[i] *= (1 + compartido)
        
        aptitud = aptitud_compartida

        # Gráfico
        plt.clf()
        plt.scatter(poblacion[:,0], poblacion[:,1], s=50)
        plt.title(f'Iteración {iteracion+1} - Generación {generacion+1}')
        plt.xlim(limite_inferior[0], limite_superior[0])
        plt.ylim(limite_inferior[1], limite_superior[1])
        plt.grid(True)
        plt.draw()
        plt.pause(0.001)

        # Selección por torneo
        pos_mejor = np.argmin(aptitud)
        mejor_aptitud[generacion] = aptitud[pos_mejor]
        mejor_individuo[generacion] = poblacion[pos_mejor]

        # Torneo
        padres = np.zeros_like(poblacion)
        torneo = np.array([np.random.permutation(num_pob),
                          np.random.permutation(num_pob)]).T
        
        for i in range(num_pob):
            a, b = torneo[i]
            if aptitud[a] < aptitud[b]:
                padres[i] = poblacion[a]
            else:
                padres[i] = poblacion[b]

        # Cruzamiento SBX
        hijos = np.zeros_like(padres)
        for i in range(0, num_pob, 2):
            if np.random.rand() <= Pc:
                hijo1, hijo2 = [], []
                for j in range(num_var):
                    p1 = padres[i, j]
                    p2 = padres[i+1, j]
                    beta = 1 + (2/(p2 - p1)) * min(p1 - limite_inferior[j],
                                                     limite_superior[j] - p2)
                    alpha = 2 - abs(beta)**-(Nc + 1)
                    U = np.random.rand()
                    
                    if U <= 1/alpha:
                        beta_c = (U * alpha)**(1/(Nc + 1))
                    else:
                        beta_c = (1/(2 - U * alpha))**(1/(Nc + 1))
                    
                    h1 = 0.5*((p1 + p2) - beta_c*abs(p2 - p1))
                    h2 = 0.5*((p1 + p2) + beta_c*abs(p2 - p1))
                    hijo1.append(h1)
                    hijo2.append(h2)
                hijos[i] = hijo1
                hijos[i+1] = hijo2
            else:
                hijos[i] = padres[i]
                hijos[i+1] = padres[i+1]

        # Mutación polinomial
        for i in range(num_pob):
            for j in range(num_var):
                if np.random.rand() <= Pm:
                    delta = min(limite_superior[j] - hijos[i,j],
                                hijos[i,j] - limite_inferior[j]) / (limite_superior[j] - limite_inferior[j])
                    r = np.random.rand()
                    
                    if r <= 0.5:
                        deltaq = (2*r + (1 - 2*r)*(1 - delta)**(Nm + 1))**(1/(Nm + 1)) - 1
                    else:
                        deltaq = 1 - (2*(1 - r) + 2*(r - 0.5)*(1 - delta)**(Nm + 1))**(1/(Nm + 1))
                    
                    hijos[i,j] += deltaq * (limite_superior[j] - limite_inferior[j])

        # Elitismo
        idx = np.random.randint(num_pob)
        hijos[idx] = mejor_individuo[generacion]
        poblacion = hijos.copy()

    # Almacenar resultados de la iteración
    mejor_idx = np.argmin(mejor_aptitud)
    resultados_generales[iteracion] = mejor_aptitud[mejor_idx]
    mejor_individuo_general[iteracion] = mejor_individuo[mejor_idx]

# Resultados finales
mejor = np.min(resultados_generales)
media = np.mean(resultados_generales)
peor = np.max(resultados_generales)
std = np.std(resultados_generales)
mejor_iter = np.argmin(resultados_generales)
mejor_x = mejor_individuo_general[mejor_iter]

print('\nResultados generales del algoritmo genético:')
print(f'Mejor aptitud: {mejor}')
print(f'Variables del mejor: {mejor_x}')
print(f'Media: {media}')
print(f'Peor: {peor}')
print(f'Desviación estándar: {std}')
%% SCRIPT DE SIMULACIÓN Y CONTROL PID PARA SEGUIMIENTO DE TRAYECTORIA
clc;
clear;
close all;

%% 1. OBTENER LA TRAYECTORIA ÓPTIMA DE REFERENCIA
% Se llama a la función modificada que ejecuta el PSO
% NOTA: Esto puede tardar varios minutos en ejecutarse
[x_ref, y_ref, v_ref] = PSO_local_global_modified();
% La trayectoria de referencia es una serie de waypoints
path_ref = [x_ref', y_ref'];

%% 2. PARÁMETROS DE LA SIMULACIÓN Y DEL VEHÍCULO
% Parámetros de simulación
dt = 0.1; % Paso de tiempo (segundos)
T_sim = 100; % Tiempo total de simulación (segundos)
N_sim = T_sim / dt; % Número de pasos

% Parámetros del vehículo (modelo cinemático de bicicleta)
L = 2.5; % Distancia entre ejes del vehículo (metros)
max_steer = deg2rad(30); % Ángulo de dirección máximo (radianes)

% Estado inicial del vehículo [x, y, yaw (orientación), velocidad]
% La orientación inicial se calcula a partir de los dos primeros puntos de la ruta
initial_yaw = atan2(y_ref(2)-y_ref(1), x_ref(2)-x_ref(1));
estado_vehiculo = [x_ref(1), y_ref(1), initial_yaw, 5]; % Inicia a 5 m/s

%% 3. PARÁMETROS DEL CONTROLADOR PID
% Ganancias del controlador (estas se deben "sintonizar" para un mejor rendimiento)
Kp = 0.7;  % Ganancia Proporcional
Ki = 0.01; % Ganancia Integral
Kd = 1.2;  % Ganancia Derivativa

% Variables del controlador
error_integral = 0;
error_previo = 0;

%% 4. BUCLE DE SIMULACIÓN
% Almacenar historial para graficar
historial_pos = zeros(N_sim, 2);
historial_cte = zeros(N_sim, 1);

fprintf('\n<strong>Iniciando simulación de seguimiento de trayectoria...</strong>\n');

for i = 1:N_sim
    % --- A. CÁLCULO DEL ERROR (Cross-Track Error - CTE) ---
    % Encontrar el punto más cercano en la trayectoria de referencia
    [~, idx_mas_cercano] = min(sum((path_ref - estado_vehiculo(1:2)).^2, 2));
    
    % Para evitar problemas en el final del recorrido, nos aseguramos de no salirnos del índice
    if idx_mas_cercano >= length(x_ref)
        idx_mas_cercano = length(x_ref) - 1;
    end
    
    % Vector del segmento de la trayectoria
    vec_path = path_ref(idx_mas_cercano+1, :) - path_ref(idx_mas_cercano, :);
    
    % Vector desde el punto de la trayectoria al vehículo
    vec_vehiculo = estado_vehiculo(1:2) - path_ref(idx_mas_cercano, :);
    
    % El CTE es la componente perpendicular. Usamos el producto cruz 2D para obtener el signo.
    % sign = x1*y2 - x2*y1
    cte = (vec_vehiculo(1) * vec_path(2) - vec_vehiculo(2) * vec_path(1)) / norm(vec_path);

    % --- B. CÁLCULO DE LA ACCIÓN DE CONTROL (PID) ---
    error_integral = error_integral + cte * dt;
    error_derivativo = (cte - error_previo) / dt;
    
    % Salida del PID
    pid_output = Kp * cte + Ki * error_integral + Kd * error_derivativo;
    
    % El ángulo de dirección (delta) es la salida del controlador
    % El signo negativo es una convención común: un error positivo (a la derecha de la ruta)
    % requiere un giro a la izquierda (ángulo de volante negativo).
    delta = -pid_output; 
    
    % Saturar el ángulo de dirección a sus límites físicos
    delta = max(-max_steer, min(max_steer, delta));
    
    % Actualizar el error previo para la siguiente iteración
    error_previo = cte;
    
    % --- C. ACTUALIZACIÓN DEL ESTADO DEL VEHÍCULO ---
    % Modelo cinemático de bicicleta
    x   = estado_vehiculo(1);
    y   = estado_vehiculo(2);
    yaw = estado_vehiculo(3);
    v   = estado_vehiculo(4); % Usamos velocidad constante por simplicidad. 
                               % Para una simulación avanzada, se podría usar v_ref(idx_mas_cercano)
                               % y un controlador de velocidad adicional.
    
    % Derivadas del estado
    x_dot   = v * cos(yaw);
    y_dot   = v * sin(yaw);
    yaw_dot = (v / L) * tan(delta);
    
    % Integración de Euler para actualizar el estado
    estado_vehiculo(1) = x + x_dot * dt;
    estado_vehiculo(2) = y + y_dot * dt;
    estado_vehiculo(3) = yaw + yaw_dot * dt;
    
    % Almacenar datos para graficar
    historial_pos(i, :) = estado_vehiculo(1:2);
    historial_cte(i) = cte;
end

fprintf('Simulación terminada.\n');

%% 5. VISUALIZACIÓN DE RESULTADOS
% Gráfico de la trayectoria
figure;
hold on;
plot(x_ref, y_ref, 'b--', 'LineWidth', 2.0, 'DisplayName', 'Trayectoria de Referencia');
plot(historial_pos(:,1), historial_pos(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Trayectoria del Vehículo');
title('Seguimiento de Trayectoria con Control PID');
xlabel('X (m)'); ylabel('Y (m)');
legend('Location', 'best');
axis equal;
grid on;

% Gráfico del error a lo largo del tiempo
figure;
plot((1:N_sim)*dt, historial_cte, 'r');
title('Error de Seguimiento (CTE) a lo largo del Tiempo');
xlabel('Tiempo (s)');
ylabel('Cross-Track Error (m)');
grid on;
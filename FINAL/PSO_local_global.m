function []=PSO_local_global()

clc;
clear;
close all;

%% Modelación de la pista

% Cargar CSV
pista = readmatrix('Norisring.csv'); % Norisring, Alemania (DTM)

% Centro de la pista
cx = pista(:,1);
cy = pista(:,2);

% Vector dirección
dx = [cx(2:end) - cx(1:end-1); cx(1) - cx(end)];
dy = [cy(2:end) - cy(1:end-1); cy(1) - cy(end)];
Vd = [dx dy];
normas = sqrt(Vd(:,1).^2 + Vd(:,2).^2);
VdU = Vd ./ normas;

% Vector normal unitario
VNormalU = [VdU(:,2) -VdU(:,1)];

% Bordes
Exterior = pista(:,3);
Interior = -pista(:,4);

BordeE = [cx cy] + VNormalU .* Exterior;
BordeI = [cx cy] + VNormalU .* Interior;

% Crear figura
figure('Units','normalized','Position',[0.2,0.1,0.5,0.8]);

% Dibujar centro y bordes
%h1 = plot(cx, cy, 'k--', 'LineWidth', 1.2); hold on;             % Centerline
h2 = plot(BordeE(:,1), BordeE(:,2), 'k', 'LineWidth', 1.2);hold on;      % Border exterior
plot(BordeI(:,1), BordeI(:,2), 'k', 'LineWidth', 1.2);     % Border interior

% Puntos de control
Pc = [1 25 50 70 90 95 100 105 110 130 150 180 185 190 195 200 205 250 290 310 320 330 335 340 350 370 382 400];

% Línea roja en la primera sección para la leyenda
idx = Pc(1);
xi = BordeI(idx,1); yi = BordeI(idx,2);
xe = BordeE(idx,1); ye = BordeE(idx,2);
h_control = plot([xi xe], [yi ye], 'r-', 'LineWidth', 1.5);  % Usado en leyenda

% Dibujar el resto de las secciones de control
for i = 2:length(Pc)
    idx = Pc(i);
    xi = BordeI(idx,1); yi = BordeI(idx,2);
    xe = BordeE(idx,1); ye = BordeE(idx,2);
    plot([xi xe], [yi ye], 'r-', 'LineWidth', 1.5);   % No entra en leyenda
end

% === Leyenda con los handles correctos ===
legend([h2, h_control], ...
    {'Border', 'Control point section'}, ...
    'Location', 'northeast','Interpreter','latex','FontSize',14);
% Títulos de los ejes con LaTeX
xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 14);

% Números de los ejes con formato LaTeX
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);
axis equal
xlabel('X'); ylabel('Y');


%% Parámetros del algoritmo

%tamaño del cúmulo
Np=200;

%numero de iteracciones maximas
Neval=200000;

%Coeficiente de aceleracion (componente cognitivo)
c1=1.5;

%coeficiente de aceleracion (componente social)
c2=2.5;

%parametro de inercia
w=0.001;

%1=global best; 2=local best(doubly-linked list)
approach=1;

%Límite de las variables
Nvar=length(Pc);
lb=Interior(Pc,:)';
ub=Exterior(Pc,:)';

%% parametros del auto
g = 9.81;           % gravedad
mu = 0.8;           % fricción neumático-pista
v_max_global = 45;  % velocidad máxima absoluta
Nt=length(cx);      %numero de puntos del recorrido

%% cúmulo de partículas inicial
rng('default');
rng('shuffle');
p=zeros(Np,Nvar);
for i=1:Nvar
    p(:,i)=lb(i)+(ub(i)-lb(i))*rand(Np,1);
end

%velocidad incial(-rango:rango)
vrang =(ub-lb);
v=repmat(-vrang,Np,1)+repmat(2*vrang,Np,1) .* rand(Np,Nvar);

%evaluacion inicial en la funcion objetivo
VFO_p=FO(p,cx,cy,Pc,VNormalU,Np,Nvar,Nt,g,mu,v_max_global);

%mejores posiciones historicas
pbest=p;
VFO_pbest=VFO_p;

%local approach (doubly-linked list)
nbh=zeros(Np,3);
nbh(:,1)=(1:Np)';
nbh(:,2)=nbh(:,1)-1;
nbh(:,3)=nbh(:,1)+1;
nbh(1,2)=nbh(end,1);
nbh(end,3)=nbh(1,1);

%inicializando iteraciones
iter=1;
while Np*iter<=Neval

    %gbest for global approach
    if approach==1
        [~,idx]=sort(VFO_pbest);
        gbest=pbest(idx(1),1:Nvar);
        VFO_gbest=VFO_pbest(idx(1))

        %gbest for global approach
    elseif approach==2
        [gbest,VFO_gbest]=funGbest(Np,Nvar,nbh,pbest,VFO_pbest);
    end

    %actualizacion de la velocidad y la posicion
    randSelf = rand(Np, Nvar);
    randSocial = rand(Np, Nvar);
    if approach==1
        v=w.*v+c1*randSelf.*(pbest-p)+c2*randSocial.*(repmat(gbest,Np,1)-p);
    elseif approach==2
        v=w.*v+c1*randSelf.*(pbest-p)+c2*randSocial.*(gbest-p);
    end
    p=p+v;

    %comprobacion de cota (aleatoria)
    ubmatrix=repmat(ub,Np,1);
    lbmatrix=repmat(lb,Np,1);
    rang=repmat(2*vrang,Np,1);
    nrand=rand(Np,Nvar);
    out=p>ubmatrix | p<lbmatrix;
    p(out)=lbmatrix(out)+nrand(out).*(ubmatrix(out)-lbmatrix(out));
    v(out)=-rang(out)+nrand(out).*rang(out);

    %evaluacion en la funcion objetivo
    VFO_p=FO(p,cx,cy,Pc,VNormalU,Np,Nvar,Nt,g,mu,v_max_global);

    %actualizacion de pbest
    [pbest,VFO_pbest]=funPbest(pbest,p,VFO_pbest,VFO_p);

    %best solution
    [~,ind]=sort(VFO_pbest);
    P_Elite=pbest(ind(1),:);
    VFO_elite=VFO_pbest(ind(1));
    [po1,po2,po3]=SalidaGrafica(P_Elite,cx,cy,Pc,VNormalU,Nvar,Nt,g,mu,v_max_global);
    delete(po1);
    delete(po2);
    delete(po3);
    %------------------------------------SALIDA---------------------------------
    formatSpec = '\n<strong>Generación</strong>: %0.0f <strong>Mejor solución f(x)</strong>: %0.6f\n';
    fprintf(formatSpec,iter,VFO_elite)
    fprintf('\n<strong>Solución</strong>:\n');
    disp(num2str(P_Elite, '%8g'))
    iter=iter+1;
end

    function [gbest,VFO_gbest]=funGbest(Np,Nvar,nbh,pbest,VFO_pbest) %gbest (DEB para los tres valores vecinos)
        gbest=zeros(Np,Nvar);
        VFO_gbest=zeros(Np,1);
        for i=1:Np
            x=[pbest(nbh(i,1),:);pbest(nbh(i,2),:);pbest(nbh(i,3),:)];
            f=[VFO_pbest(nbh(i,1));VFO_pbest(nbh(i,2));VFO_pbest(nbh(i,3))];
            [~,idx]=sort(f);
            gbest(i,1:Nvar)=x(idx(1),1:Nvar);
            VFO_gbest(i)=f(idx(1));
        end
    end
    function [p,VFO_p]=funPbest(p,V,VFO_p,VFO_V)
        mejora=VFO_V<VFO_p;
        p(mejora,:)=V(mejora,:);
        VFO_p(mejora)=VFO_V(mejora);
    end

%% objective function
    function VFO=FO(p,cx,cy,Pc,VNormalU,Np,Nvar,Nt,g,mu,v_max_global)

        VFO=zeros(Np,1);
        dt=zeros(Nt,1);

        for i=1:Np

            %ubicación de los puntos de la trayectoria en el mapa (origen el centerline cx yc)
            T=[cx(Pc) cy(Pc)]+VNormalU(Pc,:).*p(i,:)';

            %ploteo de puntos de trayectoria
            %plot(T(:,1),T(:,2),'LineStyle','none','Marker','*');

            % Curva Spline Cúbica (interpola los puntos T)

            % Agregar el primer punto al final
            T_closed = [T; T(1,:)];

            % eje independiente: usamos el índice de los puntos como "tiempo"
            t = 1:Nvar+1;
            t_eval = linspace(1, Nvar+1, Nt); % para evaluar la curva con suavidad

            % spline cúbico para X y para Y
            spline_x = makima(t, T_closed(:,1));
            spline_y = makima(t, T_closed(:,2));

            x = ppval(spline_x, t_eval);
            y = ppval(spline_y, t_eval);

            % ploteo de la curva spline
            %plot(x, y, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Spline cúbico');

            %legend({'Centro', 'Borde Izquierdo', 'Borde Derecho', 'Control Points', 'Trayectoria'}, 'Location', 'best');
            %axis equal

            %% Calculo del tiempo de recorrido
            for j = 1:Nt

                %Método del círculo circunscrito (a tres puntos consecutivos)

                % Tres puntos consecutivos
                idx1=(j-1<1)*Nt+(j-1>=1)*(j-1);
                P1 = [x(idx1) y(idx1)];
                P2 = [x(j) y(j)];
                idx3=(j+1>Nt)*1+(j+1<=Nt)*(j+1);
                P3 = [x(idx3) y(idx3)];

                % Lados del triángulo
                a = norm(P2 - P3);
                b = norm(P1 - P3);
                c = norm(P1 - P2);

                % Área del triángulo (fórmula del determinante)
                A = 0.5 * abs((P1(1)*(P2(2) - P3(2)) + P2(1)*(P3(2) - P1(2)) + P3(1)*(P1(2) - P2(2))));

                % Radio de curvatura (evita división por 0)
                r = (a * b * c) / (4 * A + 1e-16); % evitar inf en líneas rectas

                % Velocidad máxima sin perder tracción
                vmax = sqrt(r * g * mu);
                v = min(v_max_global, vmax);

                % % Etiqueta la velocidad en el punto (x(j), y(j))
                % str = sprintf('%.1f m/s', v);  % Redondea a 1 decimal
                % text(x(j), y(j), str, 'FontSize', 9, 'Color', 'b', 'VerticalAlignment', 'bottom','HorizontalAlignment', 'right');

                % Distancia entre puntos y tiempo local
                d = sqrt((x(idx3)-x(j))^2 + (y(idx3)-y(j))^2);
                dt(j) = d / v;
            end
            VFO(i,1) = sum(dt);
        end
    end

%% Salida Gráfica
    function [po1,po2,po3]=SalidaGrafica(pbest,cx,cy,Pc,VNormalU,Nvar,Nt,g,mu,v_max_global)

        dt=zeros(Nt,1);

        %ubicación de los puntos de la trayectoria en el mapa (origen el centerline cx yc)
        T=[cx(Pc) cy(Pc)]+VNormalU(Pc,:).*pbest(1,:)';

        %ploteo de puntos de trayectoria
        po1=plot(T(:,1),T(:,2),'Color','b','LineStyle','none','Marker','*');

        % Curva Spline Cúbica (interpola los puntos T)

        % Agregar el primer punto al final
        T_closed = [T; T(1,:)];

        % eje independiente: usamos el índice de los puntos como "tiempo"
        t = 1:Nvar+1;
        t_eval = linspace(1, Nvar+1, Nt); % para evaluar la curva con suavidad

        % spline cúbico para X y para Y
        spline_x = makima(t, T_closed(:,1));
        spline_y = makima(t, T_closed(:,2));

        x = ppval(spline_x, t_eval);
        y = ppval(spline_y, t_eval);

        % ploteo de la curva spline
        po2=plot(x, y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Spline cubico');

        %legend({'Centro', 'Borde Izquierdo', 'Borde Derecho', 'Control Points', 'Trayectoria'}, 'Location', 'best');
        axis equal

        %% Calculo del tiempo de recorrido
        po3(Nt,1) = gobjects;  % Prealoca vector de handles de texto
        for j = 1:Nt

            %Método del círculo circunscrito (a tres puntos consecutivos)

            % Tres puntos consecutivos
            idx1=(j-1<1)*Nt+(j-1>=1)*(j-1);
            P1 = [x(idx1) y(idx1)];
            P2 = [x(j) y(j)];
            idx3=(j+1>Nt)*1+(j+1<=Nt)*(j+1);
            P3 = [x(idx3) y(idx3)];

            % Lados del triángulo
            a = norm(P2 - P3);
            b = norm(P1 - P3);
            c = norm(P1 - P2);

            % Área del triángulo (fórmula del determinante)
            A = 0.5 * abs((P1(1)*(P2(2) - P3(2)) + P2(1)*(P3(2) - P1(2)) + P3(1)*(P1(2) - P2(2))));

            % Radio de curvatura (evita división por 0)
            r = (a * b * c) / (4 * A + 1e-16); % evitar inf en líneas rectas

            % Velocidad máxima sin perder tracción
            vmax = sqrt(r * g * mu);
            v = min(v_max_global, vmax);

            % % Etiqueta la velocidad en el punto (x(j), y(j))
            %str = sprintf('%.1f m/s', v);  % Redondea a 1 decimal
            %po3(j)=text(x(j), y(j), str, 'FontSize', 9, 'Color', 'b', 'VerticalAlignment', 'bottom','HorizontalAlignment', 'right');

            % Distancia entre puntos y tiempo local
            d = sqrt((x(idx3)-x(j))^2 + (y(idx3)-y(j))^2);
            dt(j) = d / v;
        end
    end

end






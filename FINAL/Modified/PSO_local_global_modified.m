function [x_opt, y_opt, v_opt] = PSO_local_global_modified()
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
    % Crear figura para la optimización (se cerrará y se abrirá la final)
    figure('Units','normalized','Position',[0.2,0.1,0.5,0.8]);
    h2 = plot(BordeE(:,1), BordeE(:,2), 'k', 'LineWidth', 1.2);hold on;
    plot(BordeI(:,1), BordeI(:,2), 'k', 'LineWidth', 1.2);
    % Puntos de control
    Pc = [1 25 50 70 90 95 100 105 110 130 150 180 185 190 195 200 205 250 290 310 320 330 335 340 350 370 382 400];
    idx = Pc(1);
    xi = BordeI(idx,1); yi = BordeI(idx,2);
    xe = BordeE(idx,1); ye = BordeE(idx,2);
    h_control = plot([xi xe], [yi ye], 'r-', 'LineWidth', 1.5);
    for i = 2:length(Pc)
        idx = Pc(i);
        xi = BordeI(idx,1); yi = BordeI(idx,2);
        xe = BordeE(idx,1); ye = BordeE(idx,2);
        plot([xi xe], [yi ye], 'r-', 'LineWidth', 1.5);
    end
    legend([h2, h_control], {'Border', 'Control point section'}, 'Location', 'northeast','Interpreter','latex','FontSize',14);
    xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 14);
    ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 14);
    set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 12);
    axis equal
    xlabel('X'); ylabel('Y');
    %% Parámetros del algoritmo
    Np=200;
    Neval=200000;
    c1=1.5;
    c2=2.5;

    % --- MODIFICACIÓN DE INERCIA DINÁMICA ---
    w_max = 0.9; % Inercia inicial (alta para explorar)
    w_min = 0.4; % Inercia final (baja para refinar)

    approach=1;
    Nvar=length(Pc);
    lb=Interior(Pc,:)';
    ub=Exterior(Pc,:)';
    %% parametros del auto
    g = 9.81;
    mu = 0.8;
    v_max_global = 45;
    Nt=length(cx);
    %% cúmulo de partículas inicial
    rng('default');
    rng('shuffle');
    p=zeros(Np,Nvar);
    for i=1:Nvar
        p(:,i)=lb(i)+(ub(i)-lb(i))*rand(Np,1);
    end
    vrang =(ub-lb);
    v=repmat(-vrang,Np,1)+repmat(2*vrang,Np,1) .* rand(Np,Nvar);
    VFO_p=FO(p,cx,cy,Pc,VNormalU,Np,Nvar,Nt,g,mu,v_max_global);
    pbest=p;
    VFO_pbest=VFO_p;
    nbh=zeros(Np,3);
    nbh(:,1)=(1:Np)';
    nbh(:,2)=nbh(:,1)-1;
    nbh(:,3)=nbh(:,1)+1;
    nbh(1,2)=nbh(end,1);
    nbh(end,3)=nbh(1,1);
    iter=1;
    while Np*iter<=Neval
        % --- LÍNEA AÑADIDA PARA ACTUALIZAR W ---
        % Se actualiza w en cada generación para balancear exploración/explotación
        w = w_max - (w_max - w_min) * (Np * iter) / Neval;

        if approach==1
            [~,idx]=sort(VFO_pbest);
            gbest=pbest(idx(1),1:Nvar);
            VFO_gbest=VFO_pbest(idx(1));
        elseif approach==2
            [gbest,VFO_gbest]=funGbest(Np,Nvar,nbh,pbest,VFO_pbest);
        end
        randSelf = rand(Np, Nvar);
        randSocial = rand(Np, Nvar);
        if approach==1
            v=w.*v+c1*randSelf.*(pbest-p)+c2*randSocial.*(repmat(gbest,Np,1)-p);
        elseif approach==2
            v=w.*v+c1*randSelf.*(pbest-p)+c2*randSocial.*(gbest-p);
        end
        p=p+v;
        ubmatrix=repmat(ub,Np,1);
        lbmatrix=repmat(lb,Np,1);
        rang=repmat(2*vrang,Np,1);
        nrand=rand(Np,Nvar);
        out=p>ubmatrix | p<lbmatrix;
        p(out)=lbmatrix(out)+nrand(out).*(ubmatrix(out)-lbmatrix(out));
        v(out)=-rang(out)+nrand(out).*rang(out);
        VFO_p=FO(p,cx,cy,Pc,VNormalU,Np,Nvar,Nt,g,mu,v_max_global);
        [pbest,VFO_pbest]=funPbest(pbest,p,VFO_pbest,VFO_p);
        [~,ind]=sort(VFO_pbest);
        P_Elite=pbest(ind(1),:);
        VFO_elite=VFO_pbest(ind(1));
        
        formatSpec = '\nGeneración: %0.0f Mejor solución f(x): %0.6f (w=%0.3f)\n';
        fprintf(formatSpec,iter,VFO_elite,w); % Se añade w al reporte
        iter=iter+1;
    end
    
    close(gcf); % Cierra la figura de la optimización
    
    fprintf('\n<strong>Optimización terminada. Generando trayectoria final...</strong>\n');
    [x_opt, y_opt, v_opt] = generarTrayectoria(P_Elite, cx, cy, Pc, VNormalU, Nvar, Nt, g, mu, v_max_global);
    
    figure('Units','normalized','Position',[0.2,0.1,0.5,0.8]);
    hold on;
    plot(BordeE(:,1), BordeE(:,2), 'k', 'LineWidth', 1.2);
    plot(BordeI(:,1), BordeI(:,2), 'k', 'LineWidth', 1.2);
    plot(x_opt, y_opt, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Trayectoria Óptima');
    axis equal;
    grid on;
    title('Trayectoria Óptima Final');
    xlabel('X (m)'); ylabel('Y (m)');
    legend;

    function [gbest,VFO_gbest]=funGbest(Np,Nvar,nbh,pbest,VFO_pbest)
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
    function VFO=FO(p,cx,cy,Pc,VNormalU,Np,Nvar,Nt,g,mu,v_max_global)
        VFO=zeros(Np,1);
        dt=zeros(Nt,1);
        for i=1:Np
            T=[cx(Pc) cy(Pc)]+VNormalU(Pc,:).*p(i,:)';
            T_closed = [T; T(1,:)];
            t = 1:Nvar+1;
            t_eval = linspace(1, Nvar+1, Nt);
            spline_x = makima(t, T_closed(:,1));
            spline_y = makima(t, T_closed(:,2));
            x = ppval(spline_x, t_eval);
            y = ppval(spline_y, t_eval);
            for j = 1:Nt
                idx1=(j-1<1)*Nt+(j-1>=1)*(j-1);
                P1 = [x(idx1) y(idx1)];
                P2 = [x(j) y(j)];
                idx3=(j+1>Nt)*1+(j+1<=Nt)*(j+1);
                P3 = [x(idx3) y(idx3)];
                a = norm(P2 - P3);
                b = norm(P1 - P3);
                c = norm(P1 - P2);
                A = 0.5 * abs((P1(1)*(P2(2) - P3(2)) + P2(1)*(P3(2) - P1(2)) + P3(1)*(P1(2) - P2(2))));
                r = (a * b * c) / (4 * A + 1e-16);
                vmax = sqrt(r * g * mu);
                v_current = min(v_max_global, vmax);
                d = sqrt((x(idx3)-x(j))^2 + (y(idx3)-y(j))^2);
                dt(j) = d / v_current;
            end
            VFO(i,1) = sum(dt);
        end
    end
    function [x, y, v_profile] = generarTrayectoria(pbest,cx,cy,Pc,VNormalU,Nvar,Nt,g,mu,v_max_global)
        T=[cx(Pc) cy(Pc)]+VNormalU(Pc,:).*pbest(1,:)';
        T_closed = [T; T(1,:)];
        t = 1:Nvar+1;
        t_eval = linspace(1, Nvar+1, Nt);
        spline_x = makima(t, T_closed(:,1));
        spline_y = makima(t, T_closed(:,2));
        x = ppval(spline_x, t_eval);
        y = ppval(spline_y, t_eval);
        v_profile = zeros(Nt, 1);
        for j = 1:Nt
            idx1=(j-1<1)*Nt+(j-1>=1)*(j-1);
            P1 = [x(idx1) y(idx1)];
            P2 = [x(j) y(j)];
            idx3=(j+1>Nt)*1+(j+1<=Nt)*(j+1);
            P3 = [x(idx3) y(idx3)];
            a = norm(P2 - P3); b = norm(P1 - P3); c = norm(P1 - P2);
            A = 0.5 * abs((P1(1)*(P2(2) - P3(2)) + P2(1)*(P3(2) - P1(2)) + P3(1)*(P1(2) - P2(2))));
            r = (a * b * c) / (4 * A + 1e-16);
            vmax = sqrt(r * g * mu);
            v_profile(j) = min(v_max_global, vmax);
        end
    end
end
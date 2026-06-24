clear; clc; close all;

%% --- 1. CONFIGURACIÓN DEL ROBOT (PARÁMETROS FÍSICOS) ---
L1 = 11.61;       % Altura del eslabón base fijo
L2 = 7.99;        % Longitud del eslabón rotacional 1
L3 = 8.61;        % Longitud del eslabón rotacional 2
tcp_offset = 8.5; % Distancia desde el eje horizontal hasta la punta del TCP

% Límites físicos de seguridad de los servos
lim_theta1 = [-135, 135];
lim_theta2 = [-90, 90];
lim_q3     = [0, 3.11];

%% --- 2. ENTRADA DEL ESPACIO: PUNTO INICIAL Y PUNTO FINAL DESEADO ---
% Estado Inicial del Robot (en reposo / ángulos iniciales)
th1_init = 0; 
th2_init = 0; 
q3_init  = 0;

% Calcular posición inicial cartesiana mediante Cinemática Directa
[P_init_x, P_init_y, P_init_z] = cinematica_directa(th1_init, th2_init, q3_init, L1, L2, L3, tcp_offset);
P_actual = [P_init_x; P_init_y; P_init_z];

% === COORDENADAS DESTINO (Modifica estos valores para probar otros puntos) ===
P_final = [12.0; 7.0; 2.5]; 

fprintf('--- CONFIGURACIÓN DE TRAYECTORIA ---\n');
fprintf('Punto de Partida: [%.2f, %.2f, %.2f] cm\n', P_actual(1), P_actual(2), P_actual(3));
fprintf('Punto Destino:    [%.2f, %.2f, %.2f] cm\n\n', P_final(1), P_final(2), P_final(3));

%% --- 3. RESTRICCIONES: VALIDACIÓN DE ALCANCE MÁXIMO ---
dist_plano = sqrt(P_final(1)^2 + P_final(2)^2);
if dist_plano > (L2 + L3) || dist_plano < abs(L2 - L3)
    error('ERROR FÍSICO: El punto final está fuera del alcance del brazo horizontal (L2 + L3).');
end

%% --- 4. TRAYECTORIA 1: CINEMÁTICA INVERSA PURA (PUNTO A PUNTO) ---
% Calcula los ángulos finales directamente usando las fórmulas de tu compañero
[th1_f, th2_f, q3_f] = cinematica_inversa(P_final(1), P_final(2), P_final(3), L1, L2, L3, tcp_offset, lim_theta1, lim_theta2, lim_q3);

num_pasos = 10; % Los 10 mini-pasos para que los servos aguanten suavemente
trayectoria_articular = zeros(3, num_pasos);
trayectoria_cart_CI = zeros(3, num_pasos);

for i = 1:num_pasos
    % Interpolación lineal en los motores (avanzan todos a ritmo constante)
    t = (i-1)/(num_pasos-1);
    trayectoria_articular(1,i) = th1_init + t*(th1_f - th1_init);
    trayectoria_articular(2,i) = th2_init + t*(th2_f - th2_init);
    trayectoria_articular(3,i) = q3_init + t*(q3_f - q3_init);
    
    % Calculamos por dónde pasa el TCP en el aire
    [cx, cy, cz] = cinematica_directa(trayectoria_articular(1,i), trayectoria_articular(2,i), trayectoria_articular(3,i), L1, L2, L3, tcp_offset);
    trayectoria_cart_CI(:,i) = [cx; cy; cz];
end

%% --- 5. TRAYECTORIA 2: TRAYECTORIA RECTA (JACOBIANO + MINI-PASOS) ---
trayectoria_cart_Jac = zeros(3, num_pasos);
historial_angulos_Jac = zeros(3, num_pasos);

for i = 1:num_pasos
    t = (i-1)/(num_pasos-1);
    % Dividimos el espacio cartesiano en una línea recta perfecta de 10 puntos
    P_sub = P_actual + t*(P_final - P_actual);
    trayectoria_cart_Jac(:,i) = P_sub;
    
    % Resolvemos los ángulos para este mini-punto específico
    [t1, t2, t3] = cinematica_inversa(P_sub(1), P_sub(2), P_sub(3), L1, L2, L3, tcp_offset, lim_theta1, lim_theta2, lim_q3);
    
    % --- CONTROL DE SINGULARIDADES (Tu fórmula matemática del cuaderno) ---
    t2_rad = deg2rad(t2);
    det_J = 68.7939 * sin(t2_rad); 
    
    if abs(det_J) < 0.05
        warning('ALERTA DE SINGULARIDAD en paso %d: det(J) = %.4f muy cercano a cero. Evitando área prohibida.', i, det_J);
        % Estrategia preventiva: se bloquea o satura ligeramente el ángulo para no romper los servos
        t2 = t2 + sign(t2)*0.5; 
    end
    
    historial_angulos_Jac(:,i) = [t1; t2; t3];
end

%% --- 6. ANIMACIÓN Y GRÁFICAS COMPARATIVAS (RESULTADOS Y DISCUSIÓN) ---
figure('Name', 'Analisis Cinemático SCARA: CI vs Jacobiano Lineal', 'Color', 'w', 'Position', [100, 100, 900, 500]);

for i = 1:num_pasos
    % --- SUBPLOT 1: SIMULACIÓN 3D DEL ROBOT ---
    subplot(1,2,1);
    hold off; % Limpiar gráfico para actualizar posición
    
    % Ángulos del Jacobiano en el paso actual
    t1 = historial_angulos_Jac(1,i);
    t2 = historial_angulos_Jac(2,i);
    t3 = historial_angulos_Jac(3,i);
    
    % Obtener la geometría de los eslabones
    [~, ~, ~, P0, PTB, PJ2, PJ3, PEF] = cinematica_directa(t1, t2, t3, L1, L2, L3, tcp_offset);
    
    % Graficar eslabones rígidos
    plot3([P0(1), PTB(1)], [P0(2), PTB(2)], [P0(3), PTB(3)], 'k-', 'LineWidth', 5); hold on;   
    plot3([PTB(1), PJ2(1)], [PTB(2), PJ2(2)], [PTB(3), PJ2(3)], 'b-', 'LineWidth', 5); 
    plot3([PJ2(1), PJ3(1)], [PJ2(2), PJ3(2)], [PJ2(3), PJ3(3)], 'c-', 'LineWidth', 5); 
    plot3([PJ3(1), PEF(1)], [PJ3(2), PEF(2)], [PJ3(3), PEF(3)], 'r-', 'LineWidth', 3); 
    
    % Graficar articulaciones
    plot3(P0(1), P0(2), P0(3), 'ko','MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 8); 
    plot3(PTB(1), PTB(2), PTB(3), 'bo','MarkerFaceColor', 'b', 'MarkerSize', 8);        
    plot3(PJ2(1), PJ2(2), PJ2(3), 'ro','MarkerFaceColor', 'r', 'MarkerSize', 8);        
    plot3(PJ3(1), PJ3(2), PJ3(3), 'yo','MarkerFaceColor', 'y', 'MarkerSize', 8);        
    plot3(PEF(1), PEF(2), PEF(3), 'g*', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Dibujar el histórico de las trayectorias
    plot3(trayectoria_cart_CI(1,1:i), trayectoria_cart_CI(2,1:i), trayectoria_cart_CI(3,1:i), 'm--o', 'LineWidth', 1.5);
    plot3(trayectoria_cart_Jac(1,1:i), trayectoria_cart_Jac(2,1:i), trayectoria_cart_Jac(3,1:i), 'g-s', 'LineWidth', 2);
    
    grid on; view(45, 25); axis([-20 20 -20 20 0 15]); axis equal;
    xlabel('Eje X (cm)'); ylabel('Eje Y (cm)'); zlabel('Eje Z (cm)');
    title(sprintf('Estructura SCARA - Paso %d de %d', i, num_pasos));
    legend('Base (L1)', 'Brazo 1 (L2)', 'Brazo 2 (L3)', 'Eje Z (q3)', '', '', '', '', 'TCP', 'CI Pura (Curva)', 'Jacobiano (Línea Recta)', 'Location', 'southoutside');
    
    % --- SUBPLOT 2: COMPARATIVA DE EFICIENCIA EN EL PLANO X-Y ---
    subplot(1,2,2);
    if i == 1
        hold off;
    else
        hold on;
    end
    plot(trayectoria_cart_CI(1,1:i), trayectoria_cart_CI(2,1:i), 'm--o', 'LineWidth', 1.2);
    plot(trayectoria_cart_Jac(1,1:i), trayectoria_cart_Jac(2,1:i), 'g-s', 'LineWidth', 1.5);
    grid on;
    xlabel('Posición X (cm)'); ylabel('Posición Y (cm)');
    title('Diferencia de Trayectorias en el plano X-Y');
    
    pause(0.5); % Velocidad de la animación para evaluar el comportamiento
end

fprintf('--- SIMULACIÓN FINALIZADA CON ÉXITO ---\n');


%% ==================== FUNCIONES AUXILIARES ====================

function [Px, Py, Pz, P0, PTB, PJ2, PJ3, PEF] = cinematica_directa(th1_deg, th2_deg, q3, L1, L2, L3, tcp_offset)
    % Conversión interna a radianes
    theta1 = deg2rad(th1_deg);  
    theta2 = deg2rad(th2_deg);  

    % Matrices de Denavit-Hartenberg de tu compañero
    A1 = [cos(theta1), -sin(theta1),  0,  L2*cos(theta1);
          sin(theta1),  cos(theta1),  0,  L2*sin(theta1);
          0,            0,            1,  L1;
          0,            0,            0,  1];

    A2 = [cos(theta2),  sin(theta2),  0,  L3*cos(theta2);
          sin(theta2), -cos(theta2),  0,  L3*sin(theta2); 
          0,            0,           -1,  0;
          0,            0,            0,  1];

    d3 = tcp_offset + q3;
    A3 = [1,  0,  0,  0;
          0,  1,  0,  0;
          0,  0,  1,  d3; 
          0,  0,  0,  1];

    T_total = A1 * A2 * A3;
    Px = T_total(1,4);
    Py = T_total(2,4);
    Pz = T_total(3,4);
    
    % Coordenadas para dibujar los eslabones en el espacio 3D
    P0  = [0; 0; 0];
    PTB = [0; 0; L1];
    T1  = A1; PJ2 = T1(1:3, 4);
    T2  = A1 * A2; PJ3 = T2(1:3, 4);
    PEF = [Px; Py; Pz];
end

function [th1_deg, th2_deg, q3] = cinematica_inversa(x, y, z, L1, L2, L3, tcp_offset, lim_th1, lim_th2, lim_q3)
    % --- ECUACIONES EXACTAS DE LA DIAPOSITIVA DEL COMPAÑERO ---
    
    % 1. Hallar theta 2 (con valores constantes calculados: 137.97 y 137.58)
    cos_th2 = (x^2 + y^2 - 137.97) / 137.58;
    cos_th2 = max(-1, min(1, cos_th2)); % Evita errores matemáticos si el número da 1.00001 por redondeo
    th2 = acos(cos_th2); % Configuración codo arriba (+)
    
    % 2. Hallar theta 1 (Gran división trigonométrica de la imagen)
    num_th1 = y * (7.99 + 8.61 * cos(th2)) - x * (8.61 * sin(th2));
    den_th1 = x * (7.99 + 8.61 * cos(th2)) + y * (8.61 * sin(th2));
    th1 = atan2(num_th1, den_th1); 
    
    % 3. Hallar q3 (Desplazamiento vertical)
    q3 = 3.11 - z;
    
    % Conversión a grados para el control físico de los servos
    th1_deg = rad2deg(th1);
    th2_deg = rad2deg(th2);
    
    % --- SATURACIÓN DE RESTRICCIONES FÍSICAS DE LOS MOTORES ---
    th1_deg = max(lim_th1(1), min(lim_th1(2), th1_deg));
    th2_deg = max(lim_th2(1), min(lim_th2(2), th2_deg));
    q3      = max(lim_q3(1), min(lim_q3(2), q3));
end
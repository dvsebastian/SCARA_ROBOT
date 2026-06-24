clear; clc; close all;

%% --- 1. CONFIGURACIÓN DEL ROBOT (PARÁMETROS FÍSICOS) ---
L1 = 11.61;       % Altura del eslabón base fijo
L2 = 7.99;        % Longitud del eslabón rotacional 1
L3 = 8.61;        % Longitud del eslabón rotacional 2
tcp_offset = 8.5; % Distancia desde el eje horizontal hasta la punta del TCP

% Límites físicos de seguridad de los servos (Restricciones)
lim_theta1 = [-135, 135];
lim_theta2 = [-90, 90];
lim_q3     = [0, 3.11];

%% --- 2. CONFIGURACIÓN HARDWARE Y TIEMPO (MECÁNICA A 50 Hz) ---
tiempo_trayectoria = 1.0; % Tiempo estimado que tardará el robot en moverse (1 segundo)
frecuencia_servos   = 50;  % 50 Hz mecánicos (Límite físico real de los MG995)
num_pasos = tiempo_trayectoria * frecuencia_servos; % 50 mini-pasos para fluidez perfecta

%% --- 3. ENTRADA DEL ESPACIO: CONFIGURACIÓN DEL RECORRIDO ---
% Estado Inicial del Robot (En reposo absoluto - Igual que Código 1 y 2)
th1_init = 0; 
th2_init = 0; 
q3_init  = 0;

% Calcular posición cartesiana de partida mediante Cinemática Directa
t1_i = deg2rad(th1_init); t2_i = deg2rad(th2_init);
P_init_x = L2*cos(t1_i) + L3*cos(t1_i+t2_i);
P_init_y = L2*sin(t1_i) + L3*sin(t1_i+t2_i);
P_init_z = L1 - tcp_offset - q3_init;
P_actual = [P_init_x; P_init_y; P_init_z];

% === PUNTO DESTINO IDÉNTICO AL CÓDIGO 2 ===
P_final = [12.0; 7.0; 2.5]; 

% Impresión detallada en Consola (Como el script original)
fprintf('--- VALIDACIÓN CINEMÁTICA Y RESTRICCIONES (50 Hz) ---\n');
fprintf('Frecuencia de Muestreo : %d Hz (Actualización cada 20 ms)\n', frecuencia_servos);
fprintf('Número de Mini-puntos  : %d pasos en el espacio\n', num_pasos);
fprintf('Restricciones Motor 1  : [%.1f°, %.1f°]\n', lim_theta1(1), lim_theta1(2));
fprintf('Restricciones Motor 2  : [%.1f°, %.1f°]\n', lim_theta2(1), lim_theta2(2));
fprintf('Restricciones Motor 3  : [%.1f, %.1f] cm\n\n', lim_q3(1), lim_q3(2));
fprintf('Punto de Partida Real  : Px = %.4f cm, Py = %.4f cm, Pz = %.4f cm\n', P_actual(1), P_actual(2), P_actual(3));
fprintf('Punto Destino Deseado  : Px = %.4f cm, Py = %.4f cm, Pz = %.4f cm\n', P_final(1), P_final(2), P_final(3));
fprintf('-----------------------------------------------------\n\n');

%% --- 4. TRAYECTORIA 1: CINEMÁTICA INVERSA PURA (PUNTO A PUNTO) ---
% Calcula los ángulos finales directamente para simular el "salto libre" sin control lineal
cos_th2_f = (P_final(1)^2 + P_final(2)^2 - 137.97) / 137.58;
cos_th2_f = max(-1, min(1, cos_th2_f));
th2_f_rad = acos(cos_th2_f);
num_th1_f = P_final(2)*(7.99 + 8.61*cos(th2_f_rad)) - P_final(1)*(8.61*sin(th2_f_rad));
den_th1_f = P_final(1)*(7.99 + 8.61*cos(th2_f_rad)) + P_final(2)*(8.61*sin(th2_f_rad));
th1_f_rad = atan2(num_th1_f, den_th1_f);

th1_f = rad2deg(th1_f_rad); th2_f = rad2deg(th2_f_rad); q3_f = 3.11 - P_final(3);

trayectoria_articular = zeros(3, num_pasos);
trayectoria_cart_CI = zeros(3, num_pasos);

for i = 1:num_pasos
    t = (i-1)/(num_pasos-1);
    % Movimiento interpolado en motores (Genera la curva parabólica en el aire)
    trayectoria_articular(1,i) = th1_init + t*(th1_f - th1_init);
    trayectoria_articular(2,i) = th2_init + t*(th2_f - th2_init);
    trayectoria_articular(3,i) = q3_init + t*(q3_f - q3_init);
    
    % Reconstrucción del TCP para la traza curva
    a1 = deg2rad(trayectoria_articular(1,i)); a2 = deg2rad(trayectoria_articular(2,i));
    cx = L2*cos(a1) + L3*cos(a1+a2);
    cy = L2*sin(a1) + L3*sin(a1+a2);
    cz = L1 - tcp_offset - trayectoria_articular(3,i);
    trayration_cart_CI(:,i) = [cx; cy; cz];
end

%% --- 5. TRAYECTORIA 2: TRAYECTORIA RECTA (JACOBIANO CORREGIDO + MINI-PASOS) ---
trayectoria_cart_Jac = zeros(3, num_pasos);
historial_angulos_Jac = zeros(3, num_pasos);

for i = 1:num_pasos
    t = (i-1)/(num_pasos-1);
    % Segmentación milimétrica en línea recta cartesiana
    P_sub = P_actual + t*(P_final - P_actual);
    trayectoria_cart_Jac(:,i) = P_sub;
    
    % Cinemática Inversa de tu compañero aplicada a cada mini-punto
    x = P_sub(1); y = P_sub(2); z = P_sub(3);
    cos_th2 = (x^2 + y^2 - 137.97) / 137.58;
    cos_th2 = max(-1, min(1, cos_th2));
    th2_rad = acos(cos_th2);
    
    num_th1 = y*(7.99 + 8.61*cos(th2_rad)) - x*(8.61*sin(th2_rad));
    den_th1 = x*(7.99 + 8.61*cos(th2_rad)) + y*(8.61*sin(th2_rad));
    th1_rad = atan2(num_th1, den_th1);
    
    t1 = rad2deg(th1_rad); t2 = rad2deg(th2_rad); t3 = 3.11 - z;
    
    % --- ANALISIS DE SINGULARIDAD MATEMÁTICA (Fórmula de tu cuaderno) ---
    det_J = 68.7939 * sin(th2_rad);
    if abs(det_J) < 0.05
        warning('ALERTA DE SINGULARIDAD EN VIVO: det(J) = %.4f en paso %d. Protegiendo actuadores.', det_J, i);
        t2 = t2 + sign(t2)*0.5; % Desviación controlada
    end
    
    % --- CONTROL ESTRICTO DE RESTRICCIONES ---
    t1 = max(lim_theta1(1), min(lim_theta1(2), t1));
    t2 = max(lim_theta2(1), min(lim_theta2(2), t2));
    t3 = max(lim_q3(1), min(lim_q3(2), t3));
    
    historial_angulos_Jac(:,i) = [t1; t2; t3];
end

%% --- 6. VISUALIZACIÓN DINÁMICA AVANZADA (ANIMACIÓN COMPLETA EN TIEMPO REAL) ---
figure('Name', 'Validación SCARA RRP - Control Lineal Avanzado a 50Hz', 'Color', 'w', 'Position', [100, 100, 950, 520]);

for i = 1:num_pasos
    % === SUBPLOT 1: ANIMACIÓN GEOMÉTRICA EN 3D ===
    subplot(1,2,1);
    hold off;
    
    % Ángulos calculados en el muestreo actual
    ang1 = deg2rad(historial_angulos_Jac(1,i));
    ang2 = deg2rad(historial_angulos_Jac(2,i));
    dist3 = historial_angulos_Jac(3,i);
    
    % Reconstrucción de los nodos estructurales CAD-Algebráicos
    P0  = [0; 0; 0];
    PTB = [0; 0; L1];
    PJ2 = [L2*cos(ang1); L2*sin(ang1); L1];
    PJ3 = [L2*cos(ang1) + L3*cos(ang1+ang2); L2*sin(ang1) + L3*sin(ang1+ang2); L1];
    PEF = [PJ3(1); PJ3(2); L1 - tcp_offset - dist3];
    
    % Graficación de Eslabones Rígidos
    plot3([P0(1), PTB(1)], [P0(2), PTB(2)], [P0(3), PTB(3)], 'k-', 'LineWidth', 5); hold on;   
    plot3([PTB(1), PJ2(1)], [PTB(2), PJ2(2)], [PTB(3), PJ2(3)], 'b-', 'LineWidth', 5); 
    plot3([PJ2(1), PJ3(1)], [PJ2(2), PJ3(2)], [PJ2(3), PJ3(3)], 'c-', 'LineWidth', 5); 
    plot3([PJ3(1), PEF(1)], [PJ3(2), PEF(2)], [PJ3(3), PEF(3)], 'r-', 'LineWidth', 3); 
    
    % Nodos de articulaciones y Efector Final (TCP)
    plot3(P0(1), P0(2), P0(3), 'ko', 'MarkerFaceColor', [0.2 0.2 0.2], 'MarkerSize', 8); 
    plot3(PTB(1), PTB(2), PTB(3), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);        
    plot3(PJ2(1), PJ2(2), PJ2(3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);        
    plot3(PJ3(1), PJ3(2), PJ3(3), 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 8);        
    plot3(PEF(1), PEF(2), PEF(3), 'g*', 'MarkerSize', 11, 'LineWidth', 2);
    
    % Dibujar las líneas de traza histórica acumulada en tiempo real
    plot3(trayration_cart_CI(1,1:i), trayration_cart_CI(2,1:i), trayration_cart_CI(3,1:i), 'm--', 'LineWidth', 2);
    plot3(trayectoria_cart_Jac(1,1:i), trayectoria_cart_Jac(2,1:i), trayectoria_cart_Jac(3,1:i), 'g-', 'LineWidth', 2.5);
    
    grid on; view(45, 25); axis([-20 20 -20 20 0 15]); axis equal;
    xlabel('Eje X (cm)', 'FontWeight', 'bold'); ylabel('Eje Y (cm)', 'FontWeight', 'bold'); zlabel('Eje Z (cm)', 'FontWeight', 'bold');
    title(sprintf('Estructura SCARA - Muestreo: %d / %d', i, num_pasos), 'FontSize', 11);
    legend('Columna Base (L1)', 'Brazo Horizontal (L2)', 'Brazo Horizontal (L3)', 'Varilla Prismática (TCP)', '', '', '', '', 'TCP (Efector)', 'Traza CI (Curva Libre)', 'Traza Jacobiano (Línea Recta)', 'Location', 'southoutside');
    
    % === SUBPLOT 2: ANÁLISIS DE LINEALIDAD Y DESVIACIÓN (DISCUSIÓN DE RESULTADOS) ===
    subplot(1,2,2);
    if i == 1, hold off; else hold on; end
    plot(trayration_cart_CI(1,1:i), trayration_cart_CI(2,1:i), 'm--o', 'LineWidth', 1.2, 'MarkerSize', 3);
    plot(trayectoria_cart_Jac(1,1:i), trayectoria_cart_Jac(2,1:i), 'g-s', 'LineWidth', 1.5, 'MarkerSize', 3);
    grid on;
    xlabel('Posición X (cm)'); ylabel('Posición Y (cm)');
    title('Desviación del Efector en el Plano X-Y');
    
    % Pausa calculada de 20ms (1/50 Hz) para que la velocidad en pantalla emule el hardware real
    pause(1 / frecuencia_servos); 
end

fprintf('\n--- SIMULACIÓN FINALIZADA CON ÉXITO ---\n');
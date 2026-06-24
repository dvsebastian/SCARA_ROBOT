clear; clc; close all;

L1 = 11.61; L2 = 7.99; L3 = 8.61; tcp_offset = 8.5;

% Configuración inicial (Ángulos iniciales en grados)
th_deg = [10; 20; 0]; 
t1 = deg2rad(th_deg(1)); t2 = deg2rad(th_deg(2));

% Posición inicial real del robot en el espacio
P_inicial = [L2*cos(t1) + L3*cos(t1+t2); L2*sin(t1) + L3*sin(t1+t2); L1 - tcp_offset - th_deg(3)];

% OBJETIVO DESEADO (Un paso largo para forzar el comportamiento del Jacobiano puro)
P_final = [12.0; 8.0; 2.5]; 

% Diferencial de distancia directo (Un solo gran salto)
dX = P_final - P_inicial;

% Evaluar el Jacobiano en la posición actual
J = [ -7.99*sin(t1) - 8.61*sin(t1+t2),  -8.61*sin(t1+t2),  0;
       7.99*cos(t1) + 8.61*cos(t1+t2),   8.61*cos(t1+t2),  0;
       0,                                0,               -1 ];

% Restricciones y Singularidad
det_J = 68.7939 * sin(t2);
if abs(det_J) < 0.01
    error('No se puede mover: El Jacobiano puro se indetermina en singularidades (det = 0).');
end

% --- MOVIMIENTO ARTICULAR DIRECTO POR VELOCIDAD ---
% dTh está en radianes/paso
dTh = inv(J) * dX; 

% Nuevos ángulos del robot sumando el cambio estimado
th_final_deg = th_deg + rad2deg(dTh);

% Límites físicos básicos (Restricciones mecánicas)
th_final_deg(1) = max(-135, min(135, th_final_deg(1)));
th_final_deg(2) = max(-90, min(90, th_final_deg(2)));
th_final_deg(3) = max(0, min(3.11, th_final_deg(3)));

% ¿Dónde terminó realmente tras aplicar el Jacobiano a secas?
t1_f = deg2rad(th_final_deg(1)); t2_f = deg2rad(th_final_deg(2));
P_llegada_real = [L2*cos(t1_f) + L3*cos(t1_f+t2_f); L2*sin(t1_f) + L3*sin(t1_f+t2_f); L1 - tcp_offset - th_final_deg(3)];

fprintf('--- RESULTADOS SOLO JACOBIANO (PASO ÚNICO) ---\n');
fprintf('Objetivo solicitado: X:%.2f, Y:%.2f, Z:%.2f\n', P_final(1), P_final(2), P_final(3));
fprintf('Llegada real:        X:%.2f, Y:%.2f, Z:%.2f\n', P_llegada_real(1), P_llegada_real(2), P_llegada_real(3));
fprintf('Error de posición:   %.2f cm (Efecto de no usar trayectoria)\n', norm(P_final - P_llegada_real));

% Graficar la posición del robot resultante
figure('Name','Código 2: Desplazamiento Solo Jacobiano','Color','w');
hold on; grid on; view(45, 25); axis([-20 20 -20 20 0 15]); axis equal;
P0 = [0;0;0]; PTB = [0;0;L1]; PJ2 = [L2*cos(t1_f); L2*sin(t1_f); L1]; PJ3 = [P_llegada_real(1); P_llegada_real(2); L1]; PEF = P_llegada_real;
plot3([P0(1) PTB(1)],[P0(2) PTB(2)],[P0(3) PTB(3)],'k-','LineWidth',4);
plot3([PTB(1) PJ2(1)],[PTB(2) PJ2(2)],[PTB(3) PJ2(3)],'b-','LineWidth',4);
plot3([PJ2(1) PJ3(1)],[PJ2(2) PJ3(2)],[PJ2(3) PJ3(3)],'c-','LineWidth',4);
plot3([PJ3(1) PEF(1)],[PJ3(2) PEF(2)],[PJ3(3) PEF(3)],'r-','LineWidth',2);
plot3(P_final(1), P_final(2), P_final(3), 'mo', 'MarkerSize',12, 'LineWidth',2); % Objetivo
plot3(PEF(1), PEF(2), PEF(3), 'g*', 'MarkerSize',10); % Dónde quedó
title('Código 2: Deformación/Error usando Solo Jacobiano Directo');
legend('Base','L2','L3','Eje Z','Objetivo Deseado','Posición Real Alcanzada');
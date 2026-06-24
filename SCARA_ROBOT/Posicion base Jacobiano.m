clear; clc; close all;

% Parámetros físicos del SCARA
L1 = 11.61; L2 = 7.99; L3 = 8.61; tcp_offset = 8.5;

% POSICIÓN INICIAL (Reposor / Ángulos en grados)
th1_deg = 0; 
th2_deg = 0; 
q3 = 0;

% Conversión a radianes para el Jacobiano
t1 = deg2rad(th1_deg);  
t2 = deg2rad(th2_deg);  

% --- CÁLCULO DE TU MATRIZ JACOBIANA (Fórmulas exactas de tu cuaderno) ---
J = [ -7.99*sin(t1) - 8.61*sin(t1+t2),  -8.61*sin(t1+t2),  0;
       7.99*cos(t1) + 8.61*cos(t1+t2),   8.61*cos(t1+t2),  0;
       0,                                0,               -1 ];

% CÁLCULO DEL DETERMINANTE (Tu fórmula simplificada)
det_J = 68.7939 * sin(t2);

% Posición cartesiana por Cinemática Directa
Px = L2*cos(t1) + L3*cos(t1+t2);
Py = L2*sin(t1) + L3*sin(t1+t2);
Pz = L1 - tcp_offset - q3;

% Mostrar datos en consola
fprintf('--- VALIDACIÓN EN POSICIÓN INICIAL ---\n');
fprintf('Coordenadas actuales TCP -> X: %.2f, Y: %.2f, Z: %.2f\n\n', Px, Py, Pz);
disp('Matriz Jacobiana Evaluada [J]:'); disp(J);
fprintf('Determinante del Jacobiano: %.4f\n', det_J);
if abs(det_J) < 0.05, disp('¡ALERTA! El robot inicia en una Singularidad Mecánica.'); end

% Graficar el estado estático
figure('Name','Código 1: Posición Inicial','Color','w');
hold on; grid on; view(45, 25); axis([-20 20 -20 20 0 15]); axis equal;
P0 = [0;0;0]; PTB = [0;0;L1]; PJ2 = [L2*cos(t1); L2*sin(t1); L1]; 
PJ3 = [Px; Py; L1]; PEF = [Px; Py; Pz];
plot3([P0(1) PTB(1)],[P0(2) PTB(2)],[P0(3) PTB(3)],'k-','LineWidth',4);
plot3([PTB(1) PJ2(1)],[PTB(2) PJ2(2)],[PTB(3) PJ2(3)],'b-','LineWidth',4);
plot3([PJ2(1) PJ3(1)],[PJ2(2) PJ3(2)],[PJ2(3) PJ3(3)],'c-','LineWidth',4);
plot3([PJ3(1) PEF(1)],[PJ3(2) PEF(2)],[PJ3(3) PEF(3)],'r-','LineWidth',2);
plot3(PEF(1),PEF(2),PEF(3),'g*','MarkerSize',10);
title('Código 1: Robot Estático en Posición Inicial');
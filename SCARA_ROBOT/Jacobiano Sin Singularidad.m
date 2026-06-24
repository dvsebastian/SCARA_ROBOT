clear; clc; close all;

L1 = 11.61; L2 = 7.99; L3 = 8.61; tcp_offset = 8.5;

% CONFIGURACIÓN DE COLAPSO: Brazo doblado sobre sí mismo
th1_deg = 0;
th2_deg = 180; % 180 grados exactos
q3 = 0;

t1 = deg2rad(th1_deg);  
t2 = deg2rad(th2_deg);  

% Evaluar tu determinante del cuaderno: det(J) = 68.7939 * sin(th2)
det_J = 68.7939 * sin(t2);

% Posición cartesiana resultante
Px = L2*cos(t1) + L3*cos(t1+t2);
Py = L2*sin(t1) + L3*sin(t1+t2);
Pz = L1 - tcp_offset - q3;

fprintf('--- ANÁLISIS: BRACITO PLIEGO SOBRE ESLABÓN ---\n');
fprintf('Ángulo Theta 2: %.1f°\n', th2_deg);
fprintf('Determinante calculado: %.6f (¡CERO ABSOLUTO!)\n', det_J);
fprintf('Posición de empaquetamiento -> X: %.2f, Y: %.2f, Z: %.2f\n', Px, Py, Pz);

% Graficar la colisión/plegado
figure('Name','Código 5: Singularidad Interna (180°)','Color','w');
hold on; grid on; view(45, 25); axis([-20 20 -20 20 0 15]); axis equal;

P0 = [0;0;0]; PTB = [0;0;L1]; 
PJ2 = [L2*cos(t1); L2*sin(t1); L1]; 
PJ3 = [Px; Py; L1]; PEF = [Px; Py; Pz];

% Dibujar eslabones (Verás el brazo celeste sobre el azul)
plot3([P0(1) PTB(1)],[P0(2) PTB(2)],[P0(3) PTB(3)],'k-','LineWidth',4);
plot3([PTB(1) PJ2(1)],[PTB(2) PJ2(2)],[PTB(3) PJ2(3)],'b-','LineWidth',5); % L2
plot3([PJ2(1) PJ3(1)],[PJ2(2) PJ3(2)],[PJ2(3) PJ3(3)],'c--','LineWidth',3); % L3 replegado
plot3([PJ3(1) PEF(1)],[PJ3(2) PEF(2)],[PJ3(3) PEF(3)],'r-','LineWidth',2);  % Prisma
plot3(PEF(1),PEF(2),PEF(3),'ro','MarkerFaceColor','r','MarkerSize',8);

title('Singularidad Interna: \theta_2 = 180°');
legend('Base','Brazo 1 (L2)','Brazo 2 (L3 Plegado)','Eje Z','TCP');
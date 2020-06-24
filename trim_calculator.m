% For calculating the alpha for a given elevator deflection for steady
% level flight. 
% clc;
% clear;
% close all;

load('FA_18_Parameters.mat');

points = 37;

alpha = 1:points;
del_stab = zeros(1,points);
alpha = deg2rad(alpha);

%% Zero moment calculation for alpa, del_stab

% Del_Stab - Alpha 

for i = 1:points
    Cma     =  F18Aero.Cma_0 + F18Aero.Cma_1*alpha(i) + F18Aero.Cma_2*alpha(i)^2; 
    Cmds    =  F18Aero.Cmds_0 + F18Aero.Cmds_1*alpha(i) + F18Aero.Cmds_2*alpha(i)^2; 
    del_stab(i) = -Cma/Cmds;
end

%% Lift and Drag Coefficient Calculations

C_lift = zeros(1,points);
C_drag = zeros(1,points);

for i = 1:points
    CLds = F18Aero.CLds_0 + F18Aero.CLds_1*alpha(i) + F18Aero.CLds_2*alpha(i)^2 ...
            + F18Aero.CLds_3*alpha(i)^3;
    CLa = (-0.0204 + 5.677*alpha(i) - 5.4246*alpha(i)^2 + 1.1645*alpha(i)^3);
    C_lift(i) = CLa +  CLds*del_stab(i);
end

for i = 1:points
    Cdds = F18Aero.Cdds_0 + F18Aero.Cdds_1*alpha(i)+ F18Aero.Cdds_2*alpha(i)^2 ...
           + F18Aero.Cdds_3*alpha(i)^3;  
    Cd0 = 1.5036;
    Cda = (-1.4994 - 0.1995*alpha(i) + 6.3971*alpha(i)^2 - 5.7341*alpha(i)^3 + ....
               1.4610*alpha(i)^4);
    C_drag(i) = Cda + Cd0 + Cdds*del_stab(i) ;
end

%% Thrust and Velocity Calculations

V = zeros(1,points);

for i = 1:points
    tantheta = tan(alpha(i));
    V(i) = sqrt(2*m*g/rho/S/tantheta/(C_drag(i) + C_lift(i)/tantheta));
end

T = zeros(1,points);

for i = 1:points
    costheta = cos(alpha(i));
    T(i) = C_drag(i)*0.5*rho*V(i)^2*S/costheta;
end

%% Plotting

for i = 1:points
    if del_stab(i) < -deg2rad(24)
        del_stab(i) = -deg2rad(24);
    end
end

% Elevator Angle vs Alpha
figure(1);
plot(rad2deg(alpha),rad2deg(del_stab),'LineWidth',1.5);
grid on;
hold on;
% plot(15.29,-2.606,'+');
xlabel('Alpha (in degree)');
ylabel('Elevator Deflection (in degree)');
title('Elevator deflection vs Alpha for stable level flight');
hold off;

% C_L vs Alpha
figure(2);
plot(rad2deg(alpha),C_lift,'LineWidth',1.5);
grid on;
xlabel('Alpha (in degree)');
ylabel('C_L');
title('C_L vs Alpha for stable level flight');

% C_D vs Alpha
figure(3);
plot(rad2deg(alpha),C_drag,'LineWidth',1.5);
grid on;
xlabel('Alpha (in degree)');
ylabel('C_D');
title('C_D vs Alpha for stable level flight');

% C_L vs C_D
figure(4);
plot(C_drag,C_lift,'LineWidth',1.5);
grid on;
xlabel('C_D');
ylabel('C_L');
title('C_L vs C_D');

% V vs alpha
figure(4);
plot(rad2deg(alpha),V,'LineWidth',1.5);
hold on;
% plot(15.29,350,'+');
grid on;
xlabel('Alpha (in degree)');
ylabel('V (in ft)');
title('Velocity vs alpha');

% T vs alpha
figure(5);
plot(rad2deg(alpha),T,'LineWidth',1.5);
grid on;
xlabel('Alpha (in degree)');
ylabel('T (in lbs)');
title('Thrust vs alpha');

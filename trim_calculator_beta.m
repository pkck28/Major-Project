% For calculating the alpha for a given elevator deflection for steady
% level flight. 
% clc;
% clear;
% close all;

load('FA_18_Parameters.mat');

points = 40;
points_beta = 20;

alpha = 1:points;
del_stab = zeros(1,points);
beta = -10:1:10;
alpha = deg2rad(alpha);
beta = deg2rad(beta);

%% Zero moment calculation for alpa, beta, del_stab, del_ail, del_rud

% Del_Stab - Alpha 

for i = 1:points
    Cma     =  F18Aero.Cma_0 + F18Aero.Cma_1*alpha(i) + F18Aero.Cma_2*alpha(i)^2; 
    Cmds    =  F18Aero.Cmds_0 + F18Aero.Cmds_1*alpha(i) + F18Aero.Cmds_2*alpha(i)^2; 
    del_stab(i) = -Cma/Cmds;
end

% alpha - del_rud => del_ail - beta

del_rud = zeros(points,points_beta+1);
del_ail = zeros(points,points_beta+1);

for i = 1:points
    
    a = alpha(i);
    
    Cnb     = F18Aero.Cnb_0 + F18Aero.Cnb_1*a + F18Aero.Cnb_2*a^2;
    Cndr    = F18Aero.Cndr_0 + F18Aero.Cndr_1*a + F18Aero.Cndr_2*a^2 ...
             + F18Aero.Cndr_3*a^3 + F18Aero.Cndr_4*a^4;
    Cnda    = F18Aero.Cnda_0 + F18Aero.Cnda_1*a + F18Aero.Cnda_2*a^2 ...
             + F18Aero.Cnda_3*a^3 ;   
         
    Clb     =  F18Aero.Clb_0 + F18Aero.Clb_1*a + F18Aero.Clb_2*a^2 ...
             + F18Aero.Clb_3*a^3  + F18Aero.Clb_4*a^4;
    Cldr    =  F18Aero.Cldr_0 + F18Aero.Cldr_1*a + F18Aero.Cldr_2*a^2 ...
             + F18Aero.Cldr_3*a^3;
    Clda    =  F18Aero.Clda_0 + F18Aero.Clda_1*a + F18Aero.Clda_2*a^2 ...
             + F18Aero.Clda_3*a^3;
         
    Cyb     = F18Aero.Cyb_0 + F18Aero.Cyb_1*a + F18Aero.Cyb_2*a^2 ; 
    Cyda    = F18Aero.Cyda_0 + F18Aero.Cyda_1*a + F18Aero.Cyda_2*a^2 ...
             + F18Aero.Cyda_3*a^3; 
    Cydr    = F18Aero.Cydr_0 + F18Aero.Cydr_1*a + F18Aero.Cydr_2*a^2 ...
             + F18Aero.Cydr_3*a^3;
    
    for j = 1:points_beta+1        
        del_rud(i,j) = beta(j) * (Clda*Cnb - Cnda*Clb) / ...
                        (Cnda*Cldr - Clda*Cndr);
        del_ail(i,j) = - (Clb*beta(j) + Cldr*del_rud(i,j)) / Clda;            
    end
end

%% Lift, Drag, and Sideforce Coefficient Calculations

C_lift = zeros(1,points);
C_drag = zeros(1,points);

for i = 1:points
    CLds = F18Aero.CLds_0 + F18Aero.CLds_1*alpha(i) + F18Aero.CLds_2*alpha(i)^2 ...
            + F18Aero.CLds_3*alpha(i)^3;
    CLa = (-0.0204 + 5.677*alpha(i) - 5.4246*alpha(i)^2 + 1.1645*alpha(i)^3);
    C_lift(i) = CLa*cos(2*beta(i)/3) +  CLds*del_stab(i);
end

for i = 1:points
    Cdds = F18Aero.Cdds_0 + F18Aero.Cdds_1*alpha(i)+ F18Aero.Cdds_2*alpha(i)^2 ...
           + F18Aero.Cdds_3*alpha(i)^3;  
    Cd0 = 1.5036;
    Cda = (-1.4994 - 0.1995*alpha(i) + 6.3971*alpha(i)^2 - 5.7341*alpha(i)^3 + ....
               1.4610*alpha(i)^4);
    C_drag(i) = Cda*cos(beta(i)) + Cd0 + Cdds*del_stab(i) ;
end

%% Thrust and Velocity Calculations

V = zeros(1,points);

for i = 1:points
    tantheta = tan(alpha(i));
    V(i) = sqrt(2*m*g/rho/S/tantheta/(C_drag(i) + C_lift(i)/tantheta/cos(beta(i))));
end

% T = zeros(1,points);
% 
% for i = 1:points
%     costheta = cos(alpha(i));
%     T(i) = C_drag(i)*0.5*rho*V(i)^2*S/costheta;
% end

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
plot(15.29,-2.606,'+');
xlabel('Alpha (in degree)');
ylabel('Elevator Deflection (in degree)');
title('Elevator deflection vs Alpha for stable level flight');
hold off;

% Alpha - Rudder - Beta
figure(2);

X = rad2deg(alpha);
Y = rad2deg(beta);

[X,Y] = meshgrid(Y,X);
surf(X,Y,rad2deg(del_rud));

figure(3);
surf(X,Y,rad2deg(del_ail));

% % C_L vs Alpha
% figure(2);
% plot(rad2deg(alpha),C_lift,'LineWidth',1.5);
% grid on;
% xlabel('Alpha (in degree)');
% ylabel('C_L');
% title('C_L vs Alpha for stable level flight');
% 
% % C_D vs Alpha
% figure(3);
% plot(rad2deg(alpha),C_drag,'LineWidth',1.5);
% grid on;
% xlabel('Alpha (in degree)');
% ylabel('C_D');
% title('C_D vs Alpha for stable level flight');
% 
% % C_L vs C_D
% figure(4);
% plot(C_drag,C_lift,'LineWidth',1.5);
% grid on;
% xlabel('C_D');
% ylabel('C_L');
% title('C_L vs C_D');
% 
% % V vs alpha
% figure(4);
% plot(rad2deg(alpha),V,'LineWidth',1.5);
% hold on;
% plot(15.29,350,'+');
% grid on;
% xlabel('Alpha (in degree)');
% ylabel('V (in ft)');
% title('Velocity vs alpha');
% 
% % T vs alpha
% figure(5);
% plot(rad2deg(alpha),T,'LineWidth',1.5);
% grid on;
% xlabel('Alpha (in degree)');
% ylabel('T (in lbs)');
% title('Thrust vs alpha');

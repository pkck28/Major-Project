% For calculating the trim values for steady level flight with rudder and
% elevator jam.

clc;
clear;
close all;

load('FA_18_Parameters.mat');

points = 40;
points_beta = 14;

alpha = 2:points;
del_stab = zeros(1,points-1);
beta = -7:1:7;
alpha = deg2rad(alpha);
beta = deg2rad(beta);

%% Zero moment calculation for alpa, beta, del_stab, del_ail, del_rud

% Del_Stab - Alpha 
for i = 1:points-1
    Cma     =  F18Aero.Cma_0 + F18Aero.Cma_1*alpha(i) + F18Aero.Cma_2*alpha(i)^2; 
    Cmds    =  F18Aero.Cmds_0 + F18Aero.Cmds_1*alpha(i) + F18Aero.Cmds_2*alpha(i)^2; 
    del_stab(i) = -Cma/Cmds;
end

% alpha - beta => del_ail - del_rud
del_rud = zeros(points-1,points_beta+1);
del_ail = zeros(points-1,points_beta+1);

for i = 1:points-1
    
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
    
    for j = 1:points_beta+1        
        del_rud(i,j) = beta(j) * (Clda*Cnb - Cnda*Clb) / ...
                        (Cnda*Cldr - Clda*Cndr);
        del_ail(i,j) = - (Clb*beta(j) + Cldr*del_rud(i,j)) / Clda;            
    end
end

%% Lift, Drag, and Sideforce Coefficient Calculations

C_lift = zeros(points-1,points_beta+1);
C_drag = zeros(points-1,points_beta+1);
C_Y = zeros(points-1,points_beta+1);

for i = 1:points-1
    CLds = F18Aero.CLds_0 + F18Aero.CLds_1*alpha(i) + F18Aero.CLds_2*alpha(i)^2 ...
            + F18Aero.CLds_3*alpha(i)^3;
    CLa = (-0.0204 + 5.677*alpha(i) - 5.4246*alpha(i)^2 + 1.1645*alpha(i)^3);
    for j = 1:points_beta+1        
        C_lift(i,j) = CLa*cos(2*beta(j)/3) +  CLds*del_stab(i);
    end
end

for i = 1:points-1
    Cdds = F18Aero.Cdds_0 + F18Aero.Cdds_1*alpha(i)+ F18Aero.Cdds_2*alpha(i)^2 ...
               + F18Aero.Cdds_3*alpha(i)^3;  
    Cd0 = 1.5036;
    Cda = (-1.4994 - 0.1995*alpha(i) + 6.3971*alpha(i)^2 - 5.7341*alpha(i)^3 + ....
                   1.4610*alpha(i)^4);
    for j = 1:points_beta+1        
        C_drag(i,j) = Cda*cos(beta(j)) + Cd0 + Cdds*del_stab(i);
    end
end

for i = 1:points-1
    a = alpha(i);
    Cyb     = F18Aero.Cyb_0 + F18Aero.Cyb_1*a + F18Aero.Cyb_2*a^2 ; 
    Cyda    = F18Aero.Cyda_0 + F18Aero.Cyda_1*a + F18Aero.Cyda_2*a^2 ...
             + F18Aero.Cyda_3*a^3; 
    Cydr    = F18Aero.Cydr_0 + F18Aero.Cydr_1*a + F18Aero.Cydr_2*a^2 ...
             + F18Aero.Cydr_3*a^3;
    for j = 1:points_beta+1        
        C_Y(i,j) = Cyb*beta(j) + Cyda*del_ail(i,j) + Cydr*del_rud(i,j);
    end
end

%% Thrust, Velocity and phi Calculations

phi = zeros(points-1,points_beta+1);

for i = 1:points-1
    for j = 1:points_beta+1        
        phi(i,j) = asin(( C_Y(i,j) + C_drag(i,j)*tan(beta(j)) )/C_lift(i,j) );
    end
end

V = zeros(points-1,points_beta+1);

for i = 1:points-1
    for j = 1:points_beta+1
        K = C_lift(i,j)*cos(phi(i,j)) + C_drag(i,j)*tan(alpha(i))/cos(beta(j));
        V(i,j) = sqrt(2*m*g/rho/S/K);
    end    
end

T = zeros(points-1,points_beta+1);

for i = 1:points-1
    for j = 1:points_beta+1
        T(i,j) = 0.5*rho*(V(i,j))^2*S*C_drag(i,j)/cos(alpha(i))/cos(beta(j));
    end
end

%% Plotting graphs
for i = 1:points-1
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

X = rad2deg(alpha);
Y = rad2deg(beta);
[X,Y] = meshgrid(Y,X);

% Alpha - Rudder - Beta
figure(2);
surf(X,Y,rad2deg(del_rud));
xlabel('Beta (in degree)');
ylabel('Alpha (in degree)');
zlabel('Rudder Deflection (in degree)');
title('Del_rud vs Alpha and Beta for stable level flight');

% Alpha - Aileron - Beta
figure(3);
surf(X,Y,rad2deg(del_ail));
xlabel('Beta (in degree)');
ylabel('Alpha (in degree)');
zlabel('Aileron Deflection (in degree)');
title('Del_ail vs Alpha and Beta for stable level flight');

% Alpha - Bank Angle - Beta
figure(4);
surf(X,Y,rad2deg(phi));
xlabel('Beta (in degree)');
ylabel('Alpha (in degree)');
zlabel('Bank Angle (in degree)');
title('Bank Angle vs Alpha and Beta for stable level flight');

% Alpha - Velocity - Beta
figure(5);
surf(X,Y,V);
xlabel('Beta (in degree)');
ylabel('Alpha (in degree)');
zlabel('Velocity (in ft/s)');
title('Velocity vs Alpha and Beta for stable level flight');

% Alpha - Thrust - Beta
figure(6);
surf(X,Y,T);
xlabel('Beta (in degree)');
ylabel('Alpha (in degree)');
zlabel('Thrust (in lbs)');
title('Thurst vs Alpha and Beta for stable level flight');
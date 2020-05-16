    clear;
    clc;
    close all;
    
    %opts = odeset('RelTol',1e-10,'AbsTol',1e-10);
    
    d2r = pi/180;           % Degree to radian conversion factor

    V       =  438.653328;       % Airspeed , ft/s
    beta    =  0*d2r;       % Sideslip Angle, rad
    alpha   =  10*d2r;      % Angle-of-attack, rad

    p       =  0*d2r;       % Roll rate, rad/s
    q       =  0*d2r;       % Pitch rate, rad/s
    r       =  0*d2r;       % Yaw rate, rad/s
    
    phi     =  0*d2r;       % Roll Angle, rad
    theta   =  10*d2r;       % Pitch Angle, rad
    psi     =  0*d2r;       % Yaw Angle, rad
    
    pN = 0;                 % X position in Earth Frame, ft
    pE = 0;                 % Y position in Earth Frame, ft
    h = -25000;             % Z position in Earth Frame, ft
    
    % Initial Condition for the States
    x0 = [V;beta;alpha;p;q;r;phi;theta;psi;pN;pE;h];  
    
    tfl = [0 400];                % time scale 
    u = [0;0;-2.253067*d2r;5469.045822];        % Input
    
    % Solve the Equations
    [t,y] = ode45(@FA_18_Nonlinear,tfl,x0,[],u);
    
    V = y(:,1);
    alpha = y(:,3);
    theta = y(:,8);
    
    figure(1)
    plot(t,V);
    figure(2)
    plot(t,alpha);
    hold on
    plot(t,theta);
    hold off
    legend('alpha','theta')
    
    
function [sys, x0, str,ts] = FA_18_sfun(t, x ,u, flag)

switch flag
        
    case 0 % initialize
        
        str =[];
        ts = [0 0];
        
        d2r = pi/180;           % Degree to radian conversion factor

        V       =  365.74;         % Airspeed , ft/s
        alpha   =  15*d2r;      % Angle-of-attack, rad
        beta    =  0*d2r;      % Sideslip Angle, rad

        p       =  0*d2r;      % Roll rate, rad/s
        q       =  0*d2r;       % Pitch rate, rad/s
        r       =  0*d2r;       % Yaw rate, rad/s

        phi     =  0*d2r;       % Roll Angle, rad
        theta   =  15*d2r;       % Pitch Angle, rad
        psi     =  0*d2r;       % Yaw Angle, rad

        pN = 0;                 % X position in Earth Frame, ft
        pE = 0;                 % Y position in Earth Frame, ft
        h = 25000;             % Z position in Earth Frame, ft

%         V       =  350;         % Airspeed , ft/s
%         alpha   =  40*d2r;      % Angle-of-attack, rad
%         beta    =  20*d2r;      % Sideslip Angle, rad
% 
%         p       =  10*d2r;      % Roll rate, rad/s
%         q       =  0*d2r;       % Pitch rate, rad/s
%         r       =  20*d2r;       % Yaw rate, rad/s
% 
%         phi     =  0*d2r;       % Roll Angle, rad
%         theta   =  0*d2r;       % Pitch Angle, rad
%         psi     =  0*d2r;       % Yaw Angle, rad
% 
%         pN = 0;                 % X position in Earth Frame, ft
%         pE = 0;                 % Y position in Earth Frame, ft
%         h = -25000;             % Z position in Earth Frame, ft

        % Initial Condition for the States
        x0 = [V;beta;alpha;p;q;r;phi;theta;psi;pN;pE;h];
        
        s = simsizes;
        
        s.NumContStates = 12;
        s.NumDiscStates = 0;
        s.NumOutputs = 12;
        s.NumInputs = 4;
        s.DirFeedthrough = 0;
        s.NumSampleTimes = 1; 
        
        sys = simsizes(s);
        
    case 1  % derivative
  
        sys = FA_18_Nonlinear(t,x,u);
         
    case 3  % output
        
        sys = x;
        
    case {2 4 9} 
        
        sys = [];
end
function xd = FA_18_Nonlinear(t,x,u)

    FA_18_data;

    V       =  x(1);        % Airspeed , ft/s
    beta    =  x(2);        % Sideslip Angle, rad
    alpha   =  x(3);        % Angle-of-attack, rad
    
    p       =  x(4);        % Roll rate, rad/s
    q       =  x(5);        % Pitch rate, rad/s
    r       =  x(6);        % Yaw rate, rad/s

    phi     =  x(7);        % Roll Angle, rad
    theta   =  x(8);        % Pitch Angle, rad
    psi     =  x(9);        % Yaw Angle, rad
    
    pN      =  x(10);       % X position in Earth Frame, ft
    pE      =  x(11);       % Y position in Earth Frame, ft
    h       =  x(12);       % Z position in Earth Frame, ft

%     d_AIL    = u(4);       % Aileron Deflection, rad
%     d_RUD    = u(1);       % Rudder Deflection, rad   
%     d_STAB   = u(2);       % Stabilator Deflection, rad
%     T        = u(3);       % Thrust, assumed constant
    
    d_AIL    = u(1);       % Aileron Deflection, rad
    d_RUD    = u(2);       % Rudder Deflection, rad   
    d_STAB   = u(3);       % Stabilator Deflection, rad
    T        = u(4);       % Thrust, assumed constant
    
    cosbeta = cos(beta); 
    cos2beta3 = cos(2*beta/3);
    sinbeta = sin(beta); 
    tanbeta = tan(beta);  
    secbeta = sec(beta); 

    cosalpha = cos(alpha); 
    sinalpha = sin(alpha); 

    cosphi = cos(phi); 
    sinphi = sin(phi); 

    costheta = cos(theta); 
    sintheta = sin(theta); 
    sectheta = sec(theta);
    tantheta =  tan(theta);

    cospsi = cos(psi);
    sinpsi = sin(psi);

    % Rolling Moment   
    Clb     =  F18Aero.Clb_0 + F18Aero.Clb_1*alpha + F18Aero.Clb_2*alpha^2 ...
              + F18Aero.Clb_3*alpha^3  + F18Aero.Clb_4*alpha^4;
    Cldr    =  F18Aero.Cldr_0 + F18Aero.Cldr_1*alpha + F18Aero.Cldr_2*alpha^2 ...
              + F18Aero.Cldr_3*alpha^3; 
    Clda    =  F18Aero.Clda_0 + F18Aero.Clda_1*alpha + F18Aero.Clda_2*alpha^2 ...
              + F18Aero.Clda_3*alpha^3; 
    Clp     =  F18Aero.Clp_0 + F18Aero.Clp_1*alpha; 
    Clr     =  F18Aero.Clr_0 + F18Aero.Clr_1*alpha + F18Aero.Clr_2*alpha^2;

    % ----- Total Rolling Moment
    C_l     =  Clb*beta + Clda* d_AIL + Cldr*d_RUD + Clp*p*b/2/V + Clr*r*b/2/V;
    
    % -------------------------------------------------
    %  Yawing Moment
    Cnb     = F18Aero.Cnb_0 + F18Aero.Cnb_1*alpha + F18Aero.Cnb_2*alpha^2 ; 
    Cndr    = F18Aero.Cndr_0 + F18Aero.Cndr_1*alpha + F18Aero.Cndr_2*alpha^2 ...
             + F18Aero.Cndr_3*alpha^3 + F18Aero.Cndr_4*alpha^4; 
    Cnda    = F18Aero.Cnda_0 + F18Aero.Cnda_1*alpha + F18Aero.Cnda_2*alpha^2 ...
             + F18Aero.Cnda_3*alpha^3 ; 
    Cnr     = F18Aero.Cnr_0 + F18Aero.Cnr_1*alpha; 
    Cnp     = F18Aero.Cnp_0 + F18Aero.Cnp_1*alpha; 

    % ----- Total Yawing Moment
    C_n     = Cnb*beta + Cnda*d_AIL + Cndr*d_RUD + Cnr*r*b/2/V + Cnp*p*b/2/V;

    % -------------------------------------------------
    % SideForce

    Cyb     = F18Aero.Cyb_0 + F18Aero.Cyb_1*alpha + F18Aero.Cyb_2*alpha^2 ; 
    Cyda    = F18Aero.Cyda_0 + F18Aero.Cyda_1*alpha + F18Aero.Cyda_2*alpha^2 ...
             + F18Aero.Cyda_3*alpha^3; 
    Cydr    = F18Aero.Cydr_0 + F18Aero.Cydr_1*alpha + F18Aero.Cydr_2*alpha^2 ...
             + F18Aero.Cydr_3*alpha^3; 

    % -------- Total Side Force
    C_Y     = Cyb*beta + Cydr*d_RUD +  Cyda*d_AIL;

    % -------------------------------------------------
    % Pitching Moment           
    Cma     =  F18Aero.Cma_0 + F18Aero.Cma_1*alpha + F18Aero.Cma_2*alpha^2; 
    Cmds    =  F18Aero.Cmds_0 + F18Aero.Cmds_1*alpha + F18Aero.Cmds_2*alpha^2; 
    Cmq     =  F18Aero.Cmq_0 + F18Aero.Cmq_1*alpha + F18Aero.Cmq_2*alpha^2 ...
             + F18Aero.Cmq_3*alpha^3 ;

    % --- Total Pitching Moment
    C_m     =  Cma + Cmds*d_STAB  +  Cmq*c*q/2/V;

    % -------------------------------------------------
    % Lift Coefficient
    CLds = F18Aero.CLds_0 + F18Aero.CLds_1*alpha+ F18Aero.CLds_2*alpha^2 ...
           + F18Aero.CLds_3*alpha^3; 

    C_lift = (-0.0204 + 5.677*alpha - 5.4246*alpha^2 + 1.1645*alpha^3)*cos2beta3...
         +  CLds*d_STAB;

    % -------------------------------------------------
    % Drag Coefficient
    Cdds = F18Aero.Cdds_0 + F18Aero.Cdds_1*alpha+ F18Aero.Cdds_2*alpha^2 ...
           + F18Aero.Cdds_3*alpha^3;  

    C_drag = (-1.4994 - 0.1995*alpha + 6.3971*alpha^2 - 5.7341*alpha^3 + ....
               1.4610*alpha^4) *cosbeta + 1.5036 + Cdds*d_STAB ;

    % -------------------------------------------------
    % Form Aerodynamic forces and moments

    qbar = 1/2*rho*V^2;
    L =  C_l*qbar*S*b;
    M =  C_m*qbar*S*c;
    N =  C_n*qbar*S*b;
    Y =  C_Y*qbar*S;
    Lift = C_lift*qbar*S ;
    Drag = C_drag*qbar*S ;

    % Force Equation 

    Vd =  -(Drag*cosbeta - Y*sinbeta)/m + g*(cosphi*costheta*sinalpha*cosbeta + ...
         sinphi*costheta*sinbeta -sintheta*cosalpha*cosbeta) ...
         + (T/m)*cosbeta*cosalpha;
     
    betad =  (Y*cosbeta + Drag*sinbeta)/m/V + p*sinalpha - r*cosalpha + ....
         g*costheta*sinphi*cosbeta/V + sinbeta*(g*cosalpha*sintheta...
       - g*sinalpha*cosphi*costheta + T*cosalpha/m)/V; 
     
    alphad = -Lift*secbeta/m/V + q - tanbeta*(p*cosalpha + r*sinalpha)...
              + g*(cosphi*costheta*cosalpha + sintheta*sinalpha)*secbeta/V...
              -(T*secbeta/m/V)*sinalpha;

    %--------------------------------------------------------------------------
    % Moment Equations

    pd = ((Izz* L + Ixz* N - (Ixz*(Iyy-Ixx-Izz)*p + ...
         (Ixz^2 + Izz*(Izz - Iyy))*r)*q)/(Ixx*Izz -Ixz^2));

    qd = ((M + (Izz -Ixx)*p*r + (r^2 -p^2)*Ixz)/Iyy);

    rd =  ((Ixz * L  + Ixx * N + (Ixz * (Iyy - Ixx -Izz) *r +...
          (Ixz^2 + Ixx*(Ixx - Iyy))*p)*q)/(Ixx*Izz -Ixz^2));

    %--------------------------------------------------------------------------
    % Kinetics Equation

    phid = p + (q*sinphi + r*cosphi)*tantheta;

    thetad = q*cosphi - r*sinphi;

    psid =  (q*sinphi + r*cosphi)*sectheta;
    
    %--------------------------------------------------------------------------
    % Navigation Equations

    pNd = costheta*cospsi*V*cosbeta*cosalpha + ...
          V*sinbeta*(sinphi*sintheta*cospsi - cosphi*sinpsi) + ...
          V*cosbeta*sinalpha*(cosphi*sintheta*cospsi + sinphi*sinpsi);
      
    pEd = costheta*sinpsi*V*cosbeta*cosalpha + ...
          V*sinbeta*(sinphi*sintheta*sinpsi + cosphi*cospsi) + ...
          V*cosbeta*sinalpha*(cosphi*sintheta*sinpsi + sinphi*cospsi);
    
    hd = sintheta*V*cosbeta*cosalpha - sinphi*costheta*V*sinbeta - cosphi*costheta*V*cosbeta*sinalpha;
    
    %--------------------------------------------------------------------------
    % Stacking it up in State

    xd = [Vd; betad; alphad; pd; qd; rd; phid; thetad; psid; pNd; pEd; hd];
end

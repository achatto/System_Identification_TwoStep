%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AE 4320 Assignment
% Aerodynamic Model Identification Using Two Step Approach
%  
% Abhishek Chatterjee
% 4743075
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Observation functions
function H = calc_h(X, U)
    
    g = 9.81;
    x= X(1); y = X(2); z = X(3); 
    u = X(4); v = X(5); w = X(6); 
    phi = X(7); theta = X(8); psi = X(9);
    Wx = X(10); Wy = X(11); Wz = X(12);
    lx = X(13); ly = X(14); lz = X(15); lp = X(16); lq = X(17); lr = X(18);
    
    Axm = U(1); Aym = U(2); Azm = U(3); pm = U(4); qm = U(5); rm = U(6);
    
    H(1,1) = x;
    H(2,1) = y;
    H(3,1) = z;
    H(4,1) = (u*cos(theta)+(v*sin(phi)+w*cos(phi))*sin(theta))*cos(psi)...
           -(v*cos(phi)-w*sin(phi))*sin(psi)+Wx;
    H(5,1) = (u*cos(theta)+(v*sin(phi)+w*cos(phi))*sin(theta))*sin(psi)...
           +(v*cos(phi)-w*sin(phi))*cos(psi)+Wy;
    H(6,1) = -u*sin(theta)+(v*sin(phi)+w*cos(phi))*cos(theta)+Wz;
    H(7,1) = phi;
    H(8,1) = theta;
    H(9,1) = psi;
    H(10,1) = sqrt(u^2 + v^2 + w^2);
    H(11,1) = atan(w/u);
    H(12,1) = atan(v/sqrt(u^2 + w^2));  
end
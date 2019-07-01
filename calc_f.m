%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AE 4320 Assignment
% Aerodynamic Model Identification Using Two Step Approach
%  
% Abhishek Chatterjee
% 4743075
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% State transition functions
% X = [xs; ys; zs; us; vs; ws; phis; thetas; psis; Wxs; Wys; Wzs; l_Axs; l_Ays; l_Azs; l_ps; l_qs; l_rs]
% u = [Axm]
function X_dot = calc_f(X, U)
    
    g = 9.81;
    x= X(1); y = X(2); z = X(3); 
    u = X(4); v = X(5); w = X(6); 
    phi = X(7); theta = X(8); psi = X(9);
    Wx = X(10); Wy = X(11); Wz = X(12);
    lx = X(13); ly = X(14); lz = X(15); lp = X(16); lq = X(17); lr = X(18);
    
    Axm = U(1); Aym = U(2); Azm = U(3); pm = U(4); qm = U(5); rm = U(6);
    
    X_dot(1,1) = (u*cos(theta) + (v*sin(phi) + w*cos(phi))*sin(theta))*cos(psi)...
               - (v*cos(phi) - w*sin(phi))*sin(psi) + Wx;
    X_dot(2,1) = (u*cos(theta) + (v*sin(phi) + w*cos(phi))*sin(theta))*sin(psi)...
               + (v*cos(phi) - w*sin(phi))*cos(psi) + Wy;
    X_dot(3,1) = -u*sin(theta) + (v*sin(phi) + w*cos(phi))*cos(theta) + Wz;
    X_dot(4,1) = Axm - lx - g*sin(theta) + v*(rm - lr) - w*(qm - lq);
    X_dot(5,1) = Aym - ly + g*cos(theta)*sin(phi) + w*(pm - lp) - u*(rm - lr);
    X_dot(6,1) = Azm - lz + g*cos(theta)*cos(phi) + u*(qm - lq) - v*(pm - lp);
    X_dot(7,1) = pm - lp + (qm - lq)*sin(phi)*tan(theta) + (rm - lr)*cos(phi)*tan(theta);
    X_dot(8,1) = (qm - lq)*cos(phi) - (rm - lr)*sin(phi);
    X_dot(9,1) = (qm - lq)*sin(phi)/cos(theta) + (rm - lr)*cos(phi)/cos(theta);
    X_dot(10,1) = 0;
    X_dot(11,1) = 0;
    X_dot(12,1) = 0;
    X_dot(13,1) = 0;
    X_dot(14,1) = 0;
    X_dot(15,1) = 0;
    X_dot(16,1) = 0;
    X_dot(17,1) = 0;
    X_dot(18,1) = 0;
end
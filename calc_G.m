% X = [xs; ys; zs; us; vs; ws; phis; thetas; psis; Wxs; Wys; Wzs; l_Axs; l_Ays; l_Azs; l_ps; l_qs; l_rs]
% u = [Axm]
function G = calc_G(X)
    
    g = 9.81;
    x= X(1); y = X(2); z = X(3); 
    u = X(4); v = X(5); w = X(6); 
    phi = X(7); theta = X(8); psi = X(9);
    Wx = X(10); Wy = X(11); Wz = X(12);
    lx = X(13); ly = X(14); lz = X(15); lp = X(16); lq = X(17); lr = X(18);
    
    
    G(1, :) = [0 0 0 0 0 0];
    G(2, :) = [0 0 0 0 0 0];
    G(3, :) = [0 0 0 0 0 0];
    G(4, :) = [-1 0 0 0 w -v];
    G(5, :) = [0 -1 0 -w 0 u];
    G(6, :) = [0 0 -1 v -u 0];
    G(7, :) = [0 0 0 -1 -sin(phi)*tan(theta) -cos(phi)*tan(theta)];
    G(8, :) = [0 0 0 0 -cos(phi) sin(phi)];
    G(9, :) = [0 0 0 0 -sin(phi)/cos(theta) -cos(phi)/cos(theta)];
    G(10, :) = [0 0 0 0 0 0];
    G(11, :) = [0 0 0 0 0 0];
    G(12, :) = [0 0 0 0 0 0];
    G(13, :) = [0 0 0 0 0 0];
    G(14, :) = [0 0 0 0 0 0];
    G(15, :) = [0 0 0 0 0 0];
    G(16, :) = [0 0 0 0 0 0];
    G(17, :) = [0 0 0 0 0 0];
    G(18, :) = [0 0 0 0 0 0];
end
% Generate the system and measurement noise vectors for various variables

v_x = sigma_x*randn(size(u_n));
v_y = sigma_y*randn(size(u_n));
v_z = sigma_z*randn(size(u_n));

v_u = sigma_u*randn(size(u_n));
v_v = sigma_v*randn(size(u_n));
v_w = sigma_w*randn(size(u_n));

v_phi = sigma_phi*randn(size(u_n));
v_theta = sigma_theta*randn(size(u_n));
v_psi = sigma_psi*randn(size(u_n));

v_V = sigma_V*randn(size(u_n));
v_alpha = sigma_alpha*randn(size(u_n));
v_beta = sigma_beta*randn(size(u_n));

v_Ax = sigma_Ax*randn(size(u_n));
v_Ay = sigma_Ay*randn(size(u_n));
v_Az = sigma_Az*randn(size(u_n));

v_p = sigma_p*randn(size(u_n));
v_q = sigma_q*randn(size(u_n));
v_r = sigma_r*randn(size(u_n));

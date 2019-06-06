function [X_r, U_m, Z_m, Q, R,  X_k1_k1_E0, X_std_E0] = data_preprocessing(t, u_n, v_n, w_n, phi, theta, psi, p, q, r, Ax, Ay, Az, vtas, alpha, beta)


    % calculate velocity in body frame
    ur = u_n.*cos(theta).*cos(psi) + v_n.*sin(psi).*cos(theta) - w_n.*sin(theta);
    vr = u_n.*(sin(phi).*sin(theta).*cos(psi)-cos(phi).*sin(psi))+ ...
        v_n.*(sin(phi).*sin(theta).*sin(psi)+cos(phi).*cos(psi))+ ...
        w_n.*sin(phi).*cos(theta);
    wr = u_n.*(cos(phi).*sin(theta).*cos(psi)+sin(phi).*sin(psi))+ ...
        v_n.*(cos(phi).*sin(theta).*sin(psi)-sin(phi).*cos(psi))+ ...
        w_n.*cos(phi).*cos(theta);

    %% Import Problem Parameters
    params;

    %% Generate Noise Vectors
    generate_noise_vectors;

    %% 1. Data Pre-Processing

    % compute aircraft position
    [x, y, z] = getposition(t, u_n, v_n, w_n, W_x, W_y, W_z, x0, y0, z0);

    % generate measurements
    % GPS
    x_m = x + v_x;
    y_m = y + v_y;
    z_m = z + v_z;
    u_m = u_n + W_x + v_u;
    v_m = v_n + W_y + v_v;
    w_m = w_n + W_z + v_w;
    phi_m = phi + v_phi;
    theta_m = theta + v_theta;
    psi_m = psi + v_psi;

    % Air data sensors
    V_m = vtas + v_V;
    alpha_m = alpha + v_alpha;
    beta_m = beta + v_beta;

    % IMU
    Ax_m = Ax + l_Ax + v_Ax;
    Ay_m = Ay + l_Ay + v_Ay;
    Az_m = Az + l_Az + v_Az;
    p_m = p + l_p + v_p;
    q_m = q + l_q + v_q;
    r_m = r + l_r + v_r;

    % Navigation Model
    % X_dot(t) = f[X(t), Um(t), t] + G[x(t)]w(t)
    % Z(t)     = h[X(t), Um(t), t]
    % Zm(t)    = z(t) + v(t)
    % X = [xs, ys, zs, us, vs, ws, phis, thetas, psis, W_xs, W_ys, W_zs, l_Axs, l_Ays, l_Azs, l_ps, l_qs, l_rs] 
    X_r =  [x';y';z';ur';vr';wr';phi';theta';psi';...
        W_x';W_y';W_z';l_Ax';l_Ay';l_Az';...
        l_p';l_q';l_r']; % Real state
    U_m = [Ax_m'; Ay_m'; Az_m'; p_m'; q_m'; r_m'];  % Measured inputs
    Z_m = [x_m'; y_m'; z_m'; u_m'; v_m'; w_m'; phi_m'; theta_m'; psi_m'; V_m'; alpha_m'; beta_m'];  % Measured states
end


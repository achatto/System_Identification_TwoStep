function [Xe, Ue, Ze, I, std_Xe, Xe_err, Ze_err] = kalman_filter(X_r, U_m, Z_m, Q, R, X_k1_k1_E0, X_std_E0, dt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extended Kalman Filter
% 
% AE4320 Assignment
% Abhishek Chatterjee (4743075)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialization

% Q = diag([sigma_Ax, sigma_Ay, sigma_Az, sigma_p, sigma_q, sigma_r].^2);
% R = diag([sigma_x, sigma_y, sigma_z, sigma_u, sigma_v, sigma_w...
%           sigma_phi, sigma_theta, sigma_psi, sigma_V, sigma_alpha, sigma_beta].^2);
Nx = length(X_k1_k1_E0);
N = size(Z_m, 2);

X_k1_k1 = X_k1_k1_E0;       % Initial Estimated state
P_k1_k1 = diag(X_std_E0.^2);  % Initial P - Covariance matrix

Xe = zeros(Nx,N);   % Estimated states
Ze = zeros(12,N);   % Estimated outputs
I = zeros(12,N);
std_Xe = zeros(Nx, N);   % Standard deviation in estimated state values

tk = 0;

% Kalman Filter loop
for k = 1:1:N
    % One step ahead prediction
    X_k1_k = integration_rk4(@calc_f, X_k1_k1, U_m(:, k), [tk tk+dt]);
    
    % Calculate Jacobian
    Fx = jacob(@calc_f, X_k1_k, U_m(:, k));
    Hx = jacob(@calc_h, X_k1_k, U_m(:, k));
    
    % Discretize
    G = calc_G(X_k1_k); 
    [Phi_k1_k, Gamma_k1_k] = c2d(Fx, G, dt); 
    
    % Covariance matrix of state prediction error
    P_k1_k = Phi_k1_k*P_k1_k1*Phi_k1_k' + Gamma_k1_k*Q*Gamma_k1_k';
    
    % Kalman gain
    K_k1 = P_k1_k*Hx'/(Hx*P_k1_k*Hx' + R);
    
    % Measurement update
    % innovation
    i = Z_m(:, k) - calc_h(X_k1_k, U_m(:, k));
    X_k1_k1 = X_k1_k + K_k1*i;
    
    % Covariance matrix of state estimation error
%     P_k1_k1 = (eye(Nx) - K_k1*Hx)*P_k1_k;
    P_k1_k1 = (eye(Nx) - K_k1*Hx)*P_k1_k*(eye(Nx) - K_k1*Hx)' + K_k1*R*K_k1';
    
    % Storage
    Xe(:, k) = X_k1_k1;
    std_Xe(:, k) = sqrt(diag(P_k1_k1));
    Ze(:, k) = calc_h(X_k1_k1, U_m(:, k));
    I(:, k) = i;
    % Update timer
    tk = tk + dt;
    
end

Xe_err = Xe - X_r;
Ze_err = Ze - Z_m;
Ue = U_m - Xe(13:18,:);

% generate output


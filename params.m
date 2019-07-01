%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AE 4320 Assignment
% Aerodynamic Model Identification Using Two Step Approach
%  
% Abhishek Chatterjee
% 4743075
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Script to import aircraft parameters

% Aircraft Data
Ixx = 11187.8;
Iyy = 22854.8;
Izz = 31974.8;
Ixz = 1930.1;
m = 4500;
b = 13.3250;
S = 24.99;
c = 1.9910;
rho = 1.225;


% Initial Position Values
x0 = 0;
y0 = 0;
z0 = -2000;

% Wind Amplitudes
W_x = 10*ones(size(t));
W_y = 6*ones(size(t));
W_z = 1*ones(size(t));

% Noise Parameters 
sigma_x = 10;
sigma_y = 10;
sigma_z = 10;
sigma_u = 0.1;
sigma_v = 0.1;
sigma_w = 0.1;
sigma_phi = 0.1*pi/180;
sigma_theta = 0.1*pi/180;
sigma_psi = 0.1*pi/180;
sigma_V = 0.1;
sigma_alpha = 0.1*pi/180;
sigma_beta = 0.1*pi/180;
sigma_Ax = 0.001;
sigma_Ay = 0.001;
sigma_Az = 0.001;
sigma_p = pi/180*0.001;
sigma_q = pi/180*0.001;
sigma_r = pi/180*0.001;

Q = diag([sigma_Ax, sigma_Ay, sigma_Az, sigma_p, sigma_q, sigma_r].^2);
R = diag([sigma_x, sigma_y, sigma_z, sigma_u, sigma_v, sigma_w...
          sigma_phi, sigma_theta, sigma_psi, sigma_V, sigma_alpha, sigma_beta].^2);

% Sensor bias
l_Ax = 0.001*ones(size(t));
l_Ay = 0.001*ones(size(t));
l_Az = 0.001*ones(size(t));
l_p = pi/180*0.001*ones(size(t));
l_q = pi/180*0.001*ones(size(t));
l_r = pi/180*0.001*ones(size(t));

% Kalman Filter Params
% Initial state estimates
x_E0 = 0;
y_E0 = 0;
z_E0 = -2000;
u_E0 = u_n(1)+10;
v_E0 = v_n(1)+6;
w_E0 = w_n(1)+1;
phi_E0 = 0;
theta_E0 = 0;
psi_E0 = 0;
W_x_E0 = 10;
W_y_E0 = 6;
W_z_E0 = 1;
l_Ax_E0 = 0.001;
l_Ay_E0 = 0.001;
l_Az_E0 = 0.001;
l_p_E0 = deg2rad(0.001);
l_q_E0 = deg2rad(0.001);
l_r_E0 = deg2rad(0.001);

% x_E0 = 0;
% y_E0 = 0;
% z_E0 = z0;
% u_E0 = ur(1);
% v_E0 = vr(1);
% w_E0 = wr(1);
% phi_E0 = phi(1);
% theta_E0 = theta(1);
% psi_E0 = psi(1);
% W_x_E0 = W_x(1);
% W_y_E0 = W_y(1);
% W_z_E0 = W_z(1);
% l_Ax_E0 = 0.001;
% l_Ay_E0 = 0.001;
% l_Az_E0 = 0.001;
% l_p_E0 = pi/180*0.001;
% l_q_E0 = pi/180*0.001;
% l_r_E0 = pi/180*0.001;

X_k1_k1_E0 = [x_E0; y_E0; z_E0; u_E0; v_E0; w_E0; phi_E0; theta_E0; psi_E0;...
            W_x_E0; W_y_E0; W_z_E0; l_Ax_E0; l_Ay_E0; l_Az_E0;...
            l_p_E0; l_q_E0; l_r_E0];

% Initial standard deviation estimates
x_std_E0 = 1;
y_std_E0 = 1;
z_std_E0 = 1;
u_std_E0 = 5;
v_std_E0 = 5;
w_std_E0 = 1;
phi_std_E0 = 1;
theta_std_E0 = 1;
psi_std_E0 = 1;
W_x_std_E0 = 1;
W_y_std_E0 = 1;
W_z_std_E0 = 1;
l_Ax_std_E0 = 0.01;
l_Ay_std_E0 = 0.01;
l_Az_std_E0 = 0.01;
l_p_std_E0 = 0.01;
l_q_std_E0 = 0.01;
l_r_std_E0 = 0.01;

X_std_E0 = [x_std_E0; y_std_E0; z_std_E0; u_std_E0; v_std_E0; w_std_E0;...
            phi_std_E0; theta_std_E0; psi_std_E0;...
            W_x_std_E0; W_y_std_E0; W_z_std_E0;...
            l_Ax_std_E0; l_Ay_std_E0; l_Az_std_E0;...
            l_p_std_E0; l_q_std_E0; l_r_std_E0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AE 4320 Assignment
% Aerodynamic Model Identification Using Two Step Approach
%  
% Abhishek Chatterjee
% 4743075
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Look into the func cumtrapz
close all;
clear all;

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

% Fraction of data used for identification
iden_p = 0.7;

% Flight Test Data Files

% Choose data files

%% Uncomment the following two lines for longutidinal parameter estimation
data_files = {'simdata2018/de3211'};
index = [950 2050];   % set according to datafiles being used. 950 2050 for 3211 and 950 1350 for doublet
%% Uncomment the following two lines for lateral parameter estimation
data_files = {'simdata2018/da3211', 'simdata2018/dadoublet', 'simdata2018/dr3211', 'simdata2018/drdoublet'};
index = [950 2050; 950 1200; 950 2050; 950 1200;];   % set according to datafiles being used. 950 2050 for 3211 and 950 1350 for doublet



%% Loop Through Data Files
% initialize variables

C_iden = [];
C_val = [];
S_iden = [];
S_val = [];

for file = 1:1:size(data_files, 2)
    
    % load data file
    disp('Loading Flight Test Data');
    disp(data_files{file});
    load(data_files{file});
    
    % Simulation parameters
    T = t(end);       % Simulation time
    dt = t(2)-t(1);   % time step
    N = size(t, 1);   % No. of time steps
    Nx = 18;          % No of states
    Nz = 12;          % No of measurements
    
%% 2. Data Pre-Processing
    [X_r, U_m, Z_m, Q, R,  X_k1_k1_E0, X_std_E0] = data_preprocessing(t, u_n, v_n, w_n, phi, theta, psi, p, q, r, Ax, Ay, Az, vtas, alpha, beta);
    
%% 3. Extended Kalman Filter
    [Xe, Ue, Ze, I, std_Xe, Xe_err, Ze_err] = kalman_filter(X_r, U_m, Z_m, Q, R, X_k1_k1_E0, X_std_E0, dt);
    % Plot values here

%% 4. Aerodynamic Model Identification    
    
    % Identification and validation data indices
    
    clip_idx = index(file, 2)-index(file, 1)+1;
    idx = index(file, 1) + randperm(clip_idx);
    iden_idx = idx(1:round(iden_p*clip_idx));
    val_idx  = idx(round(iden_p*clip_idx)+1:end);
    
%     iden_idx = index(file, 1):index(file, 2);
    
    t_iden = t(iden_idx);
    % Computation of forces and moments 
    [C_i, C_v] = aero_fm(dt, Ue, Ze,  m, rho, Ixx, Iyy, Izz, Ixz, b, S, c, iden_idx, val_idx); 
    
    % Computation of linear regression state values
    [S_i, S_v] = LR_states(Ue, Ze, Tc1, Tc2, de, dr, da, c, b, iden_idx, val_idx);
    
    % Data concatenation
    C_iden = [C_iden, C_i];
    C_val = [C_val, C_v];
    S_iden = [S_iden, S_i];
    S_val = [S_val, S_v]; 
    
end


%% For longutidinal parameters (Uncomment when using de3211 datafile)
% longitudinal parameter estimation (Uncomment following 3 lines when using de3211 datafile)
%  [Cx_param, Cx_err, Cx_errmean, Cx_errvar, Cx_paramcov, Cx_R2] = param_est(C_iden, S_iden, 1);
%  [Cz_param, Cz_err, Cz_errmean, Cz_errvar, Cz_paramcov, Cz_R2] = param_est(C_iden, S_iden, 2);
%  [Cm_param, Cm_err, Cm_errmean, Cm_errvar, Cm_paramcov, Cm_R2] = param_est(C_iden, S_iden, 3);
% % longutidinal parameter validation (Uncomment following 3 lines when using de3211 datafile)
%  [Cx_valerr, Cx_valerrmean, Cx_valerrvar, Cx_valR2] = param_val(C_val, S_val, Cx_param, 1);
%  [Cz_valerr, Cz_valerrmean, Cz_valerrvar, Cz_valR2] = param_val(C_val, S_val, Cz_param, 2);
%  [Cm_valerr, Cm_valerrmean, Cm_valerrvar, Cm_valR2] = param_val(C_val, S_val, Cm_param, 3);

%% For lateral parameters (Uncomment when using da3211, dadoublet, dr3211 and drdoublet datafiles)
% lateral parameter estimation (Uncomment following 3 lines)
 [Cy_param, Cy_err, Cy_errmean, Cy_errvar, Cy_paramcov, Cy_R2] = param_est(C_iden, S_iden, 4);
 [Cl_param, Cl_err, Cl_errmean, Cl_errvar, Cl_paramcov, Cl_R2] = param_est(C_iden, S_iden, 5);
 [Cn_param, Cn_err, Cn_errmean, Cn_errvar, Cn_paramcov, Cn_R2] = param_est(C_iden, S_iden, 6);
% lateral parameter validation (Uncomment following 3 lines)
 [Cy_valerr, Cy_valerrmean, Cy_valerrvar, Cy_valR2] = param_val(C_val, S_val, Cy_param, 4);
 [Cl_valerr, Cl_valerrmean, Cl_valerrvar, Cl_valR2] = param_val(C_val, S_val, Cl_param, 5);
 [Cn_valerr, Cn_valerrmean, Cn_valerrvar, Cn_valR2] = param_val(C_val, S_val, Cn_param, 6);
 
% for i = 1:1:12
%     [HH(i), PP(i)] = kstest(I(i,:)/std(I(i,:)));
% end
% HH
% PP

Cy_valR2
Cl_valR2
Cn_valR2

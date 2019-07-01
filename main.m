%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AE 4320 Assignment
% Aerodynamic Model Identification Using Two Step Approach
%  
% Abhishek Chatterjee
% 4743075
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main Script that runs all the relevant functions

%%%%%%% Instructions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plots have been commented to prevent to many plots from appearing. 
% Uncomment relevant plots or write won code to compute the plots 

% Different data files are used for the estimation of longitudinal and
% lateral parameters respectively.
% Longitudinal param estimation : Uncomment lines : 51-52, 133-135,
% 137-139, 143-154.
% Lateral param estimation : Uncomment lines : 54-55, 161-163, 165-167, 
% 169-183
% Donot uncomment both sets of the above liste lines simultaneously.

% To test the alternate model formulated in the report, replace the
% functions param_est and param_val with functions param_est_alternate and
% param_val_alternate respectively. Lines : 133-135,137-139, 145-147 
% 161-163, 165-167,181-183. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
% data_files = {'simdata2018/de3211'};
% index = [950 2050];   % set according to datafiles being used. 950 2050 for 3211 and 950 1350 for doublet
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
    
    %% EKF Convergence Analysis
%     I_mean(:,file) = mean(I);
%     I_std(:, file) = std(I);
    for i = 1:1:size(I,1)
        I_mean(i,file) = mean(I(i, :));
        I_std(i, file) = std(I(i, :));
        [I_kstest_h(i, file), I_kstest_p(i, file)] = kstest(Ze_err(i,:)/std(Ze_err(i,:))); 
    end
    % Plot values here (Comment the sections you don't want to plot)
    % Plot Computed Positions
%     figure(1);
%     subplot(3, 1, 1);
%     plot(t, X_r(1, :), 'linewidth', 1.5);
%     hold on;
%     subplot(3, 1, 2);
%     plot(t, X_r(2, :), 'linewidth', 1.5);
%     hold on;
%     subplot(3, 1, 3);
%     plot(t, X_r(3, :), 'linewidth', 1.5);
%     hold on;
%% 4. Aerodynamic Model Identification    
    
    % Identification and validation data indices
    
    clip_idx = index(file, 2)-index(file, 1)+1;
    idx = index(file, 1) + randperm(clip_idx);
    iden_idx = idx(1:round(iden_p*clip_idx));
    val_idx  = idx(round(iden_p*clip_idx)+1:end);

%     iden_idx = index(file, 1):index(file, 2);
    
    t_iden = t(iden_idx);
    % Computation of forces and moments 
    [C_i, C_v] = aero_fm(t, dt, Ue, Ze,  m, rho, Ixx, Iyy, Izz, Ixz, b, S, c, iden_idx, val_idx); 
    
    % Computation of linear regression state values
    [S_i, S_v] = LR_states(Ue, Ze, Tc1, Tc2, de, dr, da, c, b, iden_idx, val_idx);
    
    % Data concatenation
    C_iden = [C_iden, C_i];
    C_val = [C_val, C_v];
    S_iden = [S_iden, S_i];
    S_val = [S_val, S_v]; 
    
end

%% For longutidinal parameters (Uncomment when using de3211 datafile)
% % longitudinal parameter estimation (Uncomment following 3 lines when using de3211 datafile)
%  [Cx_param, Cx_err, Cx_errmean, Cx_errvar, Cx_paramcov, Cx_R2] = param_est(C_iden, S_iden, 1);
%  [Cz_param, Cz_err, Cz_errmean, Cz_errvar, Cz_paramcov, Cz_R2] = param_est(C_iden, S_iden, 2);
%  [Cm_param, Cm_err, Cm_errmean, Cm_errvar, Cm_paramcov, Cm_R2] = param_est(C_iden, S_iden, 3);
% % longutidinal parameter validation (Uncomment following 3 lines when using de3211 datafile)
%  [Cx_valerr, Cx_valerrmean, Cx_valerrvar, Cx_valR2] = param_val(C_val, S_val, Cx_param, 1);
%  [Cz_valerr, Cz_valerrmean, Cz_valerrvar, Cz_valR2] = param_val(C_val, S_val, Cz_param, 2);
%  [Cm_valerr, Cm_valerrmean, Cm_valerrvar, Cm_valR2] = param_val(C_val, S_val, Cm_param, 3);
% 
% % Standardized beta coefficient estimation
% % longitudinal 
% C_is = ((C_iden' - mean(C_iden'))./std(C_iden'))';
% S_is = ((S_iden' - mean(S_iden'))./std(S_iden'))';
% [Cx_std_param, Cx_std_err, Cx_std_errmean, Cx_std_errvar, Cx_std_paramcov, Cx_std_R2] = param_est(C_is, S_is, 1);
% [Cz_std_param, Cz_std_err, Cz_std_errmean, Cz_std_errvar, Cz_std_paramcov, Cz_std_R2] = param_est(C_is, S_is, 2);
% [Cm_std_param, Cm_std_err, Cm_std_errmean, Cm_std_errvar, Cm_std_paramcov, Cm_std_R2] = param_est(C_is, S_is, 3);
% 
% disp('Validation R^2 value for model C_x = ');
% disp(Cx_valR2);
% disp('Validation R^2 value for model C_z = ');
% disp(Cz_valR2);
% disp('Validation R^2 value for model C_m = ');
% disp(Cm_valR2);


 
 
%% For lateral parameters (Uncomment when using da3211, dadoublet, dr3211 and drdoublet datafiles)
% lateral parameter estimation (Uncomment following 3 lines)
 [Cy_param, Cy_err, Cy_errmean, Cy_errvar, Cy_paramcov, Cy_R2] = param_est(C_iden, S_iden, 4);
 [Cl_param, Cl_err, Cl_errmean, Cl_errvar, Cl_paramcov, Cl_R2] = param_est(C_iden, S_iden, 5);
 [Cn_param, Cn_err, Cn_errmean, Cn_errvar, Cn_paramcov, Cn_R2] = param_est(C_iden, S_iden, 6);
% lateral parameter validation (Uncomment following 3 lines)
 [Cy_valerr, Cy_valerrmean, Cy_valerrvar, Cy_valR2] = param_val(C_val, S_val, Cy_param, 4);
 [Cl_valerr, Cl_valerrmean, Cl_valerrvar, Cl_valR2] = param_val(C_val, S_val, Cl_param, 5);
 [Cn_valerr, Cn_valerrmean, Cn_valerrvar, Cn_valR2] = param_val(C_val, S_val, Cn_param, 6);

disp('Validation R^2 value for model C_y = ');
disp(Cy_valR2);
disp('Validation R^2 value for model C_l = ');
disp(Cl_valR2);
disp('Validation R^2 value for model C_n = ');
disp(Cn_valR2);

% Standardized beta coefficient estimation
% lateral 

C_is = ((C_iden' - mean(C_iden'))./std(C_iden'))';
S_is = ((S_iden' - mean(S_iden'))./std(S_iden'))';
[Cy_std_param, Cy_std_err, Cy_std_errmean, Cy_std_errvar, Cy_std_paramcov, Cy_std_R2] = param_est(C_is, S_is, 4);
[Cl_std_param, Cl_std_err, Cl_std_errmean, Cl_std_errvar, Cl_std_paramcov, Cl_std_R2] = param_est(C_is, S_is, 5);
[Cn_std_param, Cn_std_err, Cn_std_errmean, Cn_std_errvar, Cn_std_paramcov, Cn_std_R2] = param_est(C_is, S_is, 6);



%% Higher noise Air data sensor Results
% v_V = 1*randn(size(u_n));
% v_alpha = (pi/180)*1*randn(size(u_n));
% v_beta = (pi/180)*1*randn(size(u_n));
% 
% V_m = vtas + v_V;
% alpha_m = alpha + v_alpha;
% beta_m = beta + v_beta;
% 
% Z_m(10:12,:) = [V_m'; alpha_m'; beta_m'];    
% %% 3. Extended Kalman Filter
% [Xe1, Ue1, Ze1, I1, std_Xe1, Xe_err1, Ze_err1] = kalman_filter(X_r, U_m, Z_m, Q, R, X_k1_k1_E0, X_std_E0, dt);
% 
% %% Plot states
% figure(1);
% hold on;
% %Position
% for i = 1:1:3
% %     if(i>3)
% %         f = 180/pi;
% %     else
% %         f = 1;
% %     end
%     subplot(3,1,i);
%     plot(t, Z_m(i,:), 'yellow');
%     hold on;
%     plot(t, Xe(i,:), 'linewidth', 1.5);
%     plot(t, Xe1(i,:));
%     xlim([0 120]);
% %     ylim([-0.01 0.01]);
%     legend('Raw sensor data','Filter estimate','Filter estimate - extra noise');
%     xlabel('time (s)');
%     ylabel('x (m)');
%     set(gca, 'fontsize', 18);
%    
% end
% 
% % Velocity
% figure(2);
% hold on;
% for i = 1:1:3
% %     if(i>3)
% %         f = 180/pi;
% %     else
% %         f = 1;
% %     end
%     subplot(3,1,i);
%     plot(t, X_r(i+3,:), 'yellow', 'linewidth', 1.5);
%     hold on;
%     plot(t, Xe(i+3,:));
%     plot(t, Xe1(i+3,:));
%     xlim([0 120]);
% %     ylim([-0.01 0.01]);
%     legend('True Value','Filter estimate','Filter estimate - extra noise');
%     xlabel('time (s)');
%     ylabel('u (m/s)');
%     set(gca, 'fontsize', 18);
%     
% end
% 
% 
% % Attitude
% figure(3);
% hold on;
% for i = 1:1:3
% %     if(i>3)
% %         f = 180/pi;
% %     else
% %         f = 1;
% %     end
%     subplot(3,1,i);
%     plot(t, Z_m(i+6,:)*180/pi, 'yellow');
%     hold on;
%     plot(t, Xe(i+6,:)*180/pi, 'linewidth', 1.5);
%     plot(t, Xe1(i+6,:)*180/pi, 'linewidth', 1.5);
%     xlim([0 120]);
% %     ylim([-0.01 0.01]);
%     legend('Raw sensor data','Filter estimate','Filter estimate - extra noise');
%     xlabel('time (s)');
%     ylabel('\phi (deg)');
%     set(gca, 'fontsize', 18);
%     
% end
% 
% % Wind Velocities
% figure(4);
% hold on;
% for i = 1:1:3
% %     if(i>3)
% %         f = 180/pi;
% %     else
% %         f = 1;
% %     end
%     subplot(3,1,i);
%     plot(t, X_r(i+9,:), 'yellow', 'linewidth', 1.5);
%     hold on;
%     plot(t, Xe(i+9,:));
%     plot(t, Xe1(i+9,:));
%     xlim([0 120]);
% %     ylim([-0.01 0.01]);
%     legend('True Value','Filter estimate','Filter estimate - extra noise');
%     xlabel('time (s)');
%     ylabel('\phi (deg)');
%     set(gca, 'fontsize', 18);    
% end
% 
% 
% %% Plot bias estimates
% figure(1);
% hold on;
% for i = 1:1:6
%     if(i>3)
%         f = 180/pi;
%     else
%         f = 1;
%     end
%     subplot(2,3,i);
%     plot(t, f*Xe(12+i,:));
%     hold on;
%     plot(t, f*Xe1(12+i,:));
%     plot(t, 0.001*ones(size(t)));
%     xlim([0 120]);
%     ylim([-0.01 0.01]);
%     legend('Filter estimate','Filter estimate - extra noise', 'True value');
%     xlabel('time (s)');
%     ylabel('\lambda_x (m/s^2)');
%     set(gca, 'fontsize', 18);
% end


%% Sensor measurement plot
% figure(1);
% hold on;
% for i = 1:1:size(Z_m, 1)-3
%     if(i>=7)
%         f = 180/pi;
%     else
%         f = 1;
%     end
%     subplot(3,3,i);
%     plot(t, f*Z_m(i,:));
%     xlim([0 120]);
%     xlabel('time (s)');
%     ylabel('x_{GPS}_m (m)');
%     set(gca, 'fontsize', 18);
% end
% figure(1);
% hold on;
% for i = 10:1:size(Z_m, 1)
%     if(i>=11)
%         f = 180/pi;
%     else
%         f = 1;
%     end
%     subplot(3,1,(i-9));
%     plot(t, f*Z_m(i,:));
%     xlim([0 120]);
%     xlabel('time (s)');
%     ylabel('V_{GPS}_m (m/s)');
%     set(gca, 'fontsize', 18);
% end
% 
% figure(1);
% hold on;
% for i = 1:1:size(U_m, 1)
%     if(i>3)
%         f = 180/pi;
%     else
%         f = 1;
%     end
%     subplot(3,2,i);
%     plot(t, f*U_m(i,:));
%     xlim([0 120]);
%     xlabel('time (s)');
%     ylabel('A_x_m (m/s)');
%     set(gca, 'fontsize', 18);
% end


%%  Calculated Position plot
% subplot(3, 1, 1);
% xlabel('time (s)');1
% ylabel('x position (m)');
% legend('de3211', 'da3211', 'dadoublet', 'dr3211', 'drdoublet');
% set(gca, 'fontsize', 18);
% 
% subplot(3, 1, 2);
% xlabel('time (s)');
% ylabel('y position (m)');
% legend('de3211', 'da3211', 'dadoublet', 'dr3211', 'drdoublet');
% set(gca, 'fontsize', 18);
% subplot(3, 1, 3);
% xlabel('time (s)');
% ylabel('z position (m)');
% legend('de3211', 'da3211', 'dadoublet', 'dr3211', 'drdoublet');
% set(gca, 'fontsize', 18);




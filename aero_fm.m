%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AE 4320 Assignment
% Aerodynamic Model Identification Using Two Step Approach
%  
% Abhishek Chatterjee
% 4743075
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Compute Aerodynamic Forces and Moments


function [C_i, C_v] = aero_fm(t, dt, Ue, Ze, m, rho, Ixx, Iyy, Izz, Ixz, b, S, c, i_idx, v_idx)

Ue_temp = Ue(:, 3:end);
Ue_temp = [Ue_temp zeros(size(Ue,1),2)];
Ued = (Ue_temp-Ue)./(2*dt);
Ued(:,1) = (Ue(:,2)-Ue(:,1))./(dt);
Ued(:,end) = (Ue(:,end)-Ue(:,end-1))./(dt);

%% Dimensionless Aerodynamic Force Body Components

Cx = m.*Ue(1,:)./(0.5.*rho.*Ze(10,:).^2.*S);
Cy = m.*Ue(2,:)./(0.5.*rho.*Ze(10,:).^2.*S);
Cz = m.*Ue(3,:)./(0.5.*rho.*Ze(10,:).^2.*S);

%% Dimensionless Aerodynamic Moment Components

Cl = (Ued(4,:).*Ixx + Ue(5,:).*Ue(6,:).*(Izz - Iyy) - (Ue(4,:).*Ue(5,:) + Ued(6,:)).*Ixz)./(0.5.*rho.*Ze(10,:).^2.*S.*b);
Cm = (Ued(5,:).*Iyy + Ue(6,:).*Ue(4,:).*(Ixx - Izz) + (Ue(4,:).^2 - Ued(6,:).^2).*Ixz)./(0.5.*rho.*Ze(10,:).^2.*S.*c);
Cn = (Ued(6,:).*Izz + Ue(4,:).*Ue(5,:).*(Iyy - Ixx) + (Ue(5,:).*Ue(6,:) - Ued(4,:)).*Ixz)./(0.5.*rho.*Ze(10,:).^2.*S.*b);

C_i = [Cx(i_idx); Cy(i_idx); Cz(i_idx); Cl(i_idx); Cm(i_idx); Cn(i_idx)];
C_v = [Cx(v_idx); Cy(v_idx); Cz(v_idx); Cl(v_idx); Cm(v_idx); Cn(v_idx)];

%% Plots
% figure(1);
% subplot(3,1,1);
% plot(t, Cx);
% xlabel('time (s)');
% ylabel('C_x');
% set(gca, 'fontsize', 18);
% subplot(3,1,2);
% plot(t, Cy);
% xlabel('time (s)');
% ylabel('C_y');
% set(gca, 'fontsize', 18);
% subplot(3,1,3);
% plot(t, Cz);
% xlabel('time (s)');
% ylabel('C_y');
% set(gca, 'fontsize', 18);
% 
% figure(2);
% subplot(3,1,1);
% plot(t, Cl);
% xlabel('time (s)');
% ylabel('C_l');
% set(gca, 'fontsize', 18);
% subplot(3,1,2);
% plot(t, Cm);
% xlabel('time (s)');
% ylabel('C_m');
% set(gca, 'fontsize', 18);
% subplot(3,1,3);
% plot(t, Cn);
% xlabel('time (s)');
% ylabel('C_n');
% set(gca, 'fontsize', 18);
end

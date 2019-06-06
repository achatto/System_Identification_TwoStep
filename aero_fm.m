%% Compute derivatives of accelerations and angular rates

% Ued = zeros(size(Ue));
function [C_i, C_v] = aero_fm(dt, Ue, Ze, m, rho, Ixx, Iyy, Izz, Ixz, b, S, c, i_idx, v_idx)

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

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AE 4320 Assignment
% Aerodynamic Model Identification Using Two Step Approach
%  
% Abhishek Chatterjee
% 4743075
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Function to calculate the values of regression state variables

function [S_i, S_v] = LR_states(Ue, Ze, Tc1, Tc2, de, dr, da, c, b, i_idx, v_idx);

Alpha = Ze(11,:);
Alpha2 = Alpha.^2;
Qcv = Ue(5,:).*c./Ze(10,:);
de = de';
Tc = (Tc1+Tc2)';

Beta = Ze(12, :);
Pbv = Ue(4,:).*b./(2*Ze(10,:));
Rbv = Ue(6,:).*b./(2*Ze(10,:));
da = da';
dr = dr';

S_i = [Alpha(i_idx); Alpha2(i_idx); Qcv(i_idx); de(i_idx); Tc(i_idx); Beta(i_idx); Pbv(i_idx); Rbv(i_idx); da(i_idx); dr(i_idx)];
S_v = [Alpha(v_idx); Alpha2(v_idx); Qcv(v_idx); de(v_idx); Tc(v_idx); Beta(v_idx); Pbv(v_idx); Rbv(v_idx); da(v_idx); dr(v_idx)];

end
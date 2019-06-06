function [param, err, err_mean, err_var, param_cov, R2] = param_est(C_i, S_i, flag)

% flag = 1 Cx parameter identification
% flag = 2 Cz parameter identification
% flag = 3 Cm parameter identification
% flag = 4 Cy parameter identification
% flag = 5 Cl parameter identification
% flag = 6 Cn parameter identification

% For index reference
% S_i = [Alpha(i_idx); Alpha2(i_idx); Qcv(i_idx); de(i_idx); Tc(i_idx); Beta(i_idx); Pbv(i_idx); Rbv(i_idx); da(i_idx); dr(i_idx)];
% C_i = [Cx(i_idx); Cy(i_idx); Cz(i_idx); Cl(i_idx); Cm(i_idx); Cn(i_idx)];
    
    if(flag == 1)  % if Cx parameters have to be identified
        y = C_i(1, :)';
        A = [ones(size(y)), S_i(1, :)', S_i(2, :)', S_i(3, :)', S_i(4, :)', S_i(5, :)'];
    elseif(flag == 2) % if Cz parameters have to be identified
        y = C_i(3, :)';
        A = [ones(size(y)), S_i(1, :)', S_i(3, :)', S_i(4, :)', S_i(5, :)'];
    elseif(flag == 3) % if Cm parameters have to be identified
        y = C_i(5, :)';
        A = [ones(size(y)), S_i(1, :)', S_i(3, :)', S_i(4, :)', S_i(5, :)'];
    elseif(flag == 4) % if Cy parameters have to be identified
        y = C_i(2, :)';
        A = [ones(size(y)), S_i(6, :)', S_i(7, :)', S_i(8, :)', S_i(9, :)', S_i(10, :)'];
    elseif(flag == 5) % if Cl parameters have to be identified
        y = C_i(4, :)';
        A = [ones(size(y)), S_i(6, :)', S_i(7, :)', S_i(8, :)', S_i(9, :)', S_i(10, :)'];
    elseif(flag == 6) % if Cn parameters have to be identified
        y = C_i(6, :)';
        A = [ones(size(y)), S_i(6, :)', S_i(7, :)', S_i(8, :)', S_i(9, :)', S_i(10, :)'];
    end
    
% parameter estimation
    
    param =  (A'*A)\A'*y;
%     param = A\y;
    err = y - A*param;
    err_mean = mean(err);
    err_var = var(err);
    param_cov = err_var*(inv(A'*A));
    R2 = 1 - sum(err.^2)/sum((y - mean(y)).^2); 
        
end


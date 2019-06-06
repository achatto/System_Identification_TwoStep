function [err, err_mean, err_var, R2] = param_val(C_v, S_v, C_param, flag)

if(flag == 1)  % if Cx parameters have to be validated
        y = C_v(1, :)';
        A = [ones(size(y)), S_v(1, :)', S_v(2, :)', S_v(3, :)', S_v(4, :)', S_v(5, :)'];
    elseif(flag == 2) % if Cz parameters have to be validated
        y = C_v(3, :)';
        A = [ones(size(y)), S_v(1, :)', S_v(3, :)', S_v(4, :)', S_v(5, :)'];
    elseif(flag == 3) % if Cm parameters have to be validated
        y = C_v(5, :)';
        A = [ones(size(y)), S_v(1, :)', S_v(3, :)', S_v(4, :)', S_v(5, :)'];
    elseif(flag == 4) % if Cy parameters have to be validated
        y = C_v(2, :)';
        A = [ones(size(y)), S_v(6, :)', S_v(7, :)', S_v(8, :)', S_v(9, :)', S_v(10, :)'];
    elseif(flag == 5) % if Cl parameters have to be validated
        y = C_v(4, :)';
        A = [ones(size(y)), S_v(6, :)', S_v(7, :)', S_v(8, :)', S_v(9, :)', S_v(10, :)'];
    elseif(flag == 6) % if Cn parameters have to be validated
        y = C_v(6, :)';
        A = [ones(size(y)), S_v(6, :)', S_v(7, :)', S_v(8, :)', S_v(9, :)', S_v(10, :)'];
end
    

%     param = A\y;
    err = y - A*C_param;
    err_mean = mean(err);
    err_var = var(err);
    R2 = 1 - sum(err.^2)/sum((y - mean(y)).^2);
    
end

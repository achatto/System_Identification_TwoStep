%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AE 4320 Assignment
% Aerodynamic Model Identification Using Two Step Approach
%  
% Abhishek Chatterjee
% 4743075
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Jacobian Computation
function J = jacob(f, x0, u0)
    
    h = 1e-6;
    l_x0=length(x0);
    f0=feval(f,x0,u0);
    l_f=size(f0,1);
    J = zeros(l_f, l_x0);
    
    for i=1:l_x0
        dx = [ zeros(i-1,1); h; zeros(l_x0-i,1)];
        J(:,i) = ( feval(f,x0+dx, u0) - feval(f,x0-dx, u0))/(2*h);
    end
    

end
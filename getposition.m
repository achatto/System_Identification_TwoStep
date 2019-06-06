function [x, y, z] = getposition(t, u_n, v_n, w_n, W_x, W_y, W_z, x0, y0, z0)

    % Initialization
    x = zeros(size(t));
    y = zeros(size(t));
    z = zeros(size(t));

    % Initial Values
    x(1) = x0;
    y(1) = y0;
    z(1) = z0;
    
    % Speed with wind
    ue = u_n + W_x;
    ve = v_n + W_y;
    we = w_n + W_z;
    
    % Integration    
    for i = 2:1:numel(t)
        dt = t(i) - t(i-1);
        x(i) =  x(i-1) + dt*(ue(i) + ue(i-1))/2; 
        y(i) =  y(i-1) + dt*(ve(i) + ve(i-1))/2;
        z(i) =  z(i-1) + dt*(we(i) + we(i-1))/2;
    end
    
    % Plot
%     if (flag == 1)
%         figure;
%         subplot(3,1,1);
%         plot(t, x);
%         xlabel('time(s)');
%         ylabel('x_e (m)');
%         set(gca, 'fontsize', 15);
%         
%         subplot(3,1,2);
%         plot(t, y);
%         xlabel('time(s)');
%         ylabel('y_e (m)');
%         set(gca, 'fontsize', 15);
%         
%         subplot(3,1,3);
%         plot(t, z);
%         xlabel('time(s)');
%         ylabel('z_e (m)');
%         set(gca, 'fontsize', 15);
%         suptitle('Aircraft Position in Earth Reference Frame');
        
%     end
end
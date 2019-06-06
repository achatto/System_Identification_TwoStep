function [Ued] = U_derivatives(dt, Ue)
    
    Ue_temp = Ue(:, 3:end);
    Ue_temp = [Ue_temp zeros(size(Ue,1),2)];
    Ued = (Ue_temp-Ue)./(2*dt);
    Ued(:,1) = (Ue(:,2)-Ue(:,1))./(dt);
    Ued(:,end) = (Ue(:,end)-Ue(:,end-1))./(dt);
    
end
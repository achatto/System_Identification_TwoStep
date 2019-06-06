function [w] = integration_rk4(fn, xin, uin, t)
    g = 9.81; 
    a = t(1); 
    b = t(2);
    w = xin;
    N = 2;
    h = (b-a) / N;
    t = a;

    for j=1:N
        K1 = h * fn(w, uin);
        K2 = h * fn(w+K1/2, uin);
        K3 = h * fn(w+K2/2, uin);
        K4 = h * fn(w+K3, uin);

        w = w + (K1 + 2*K2 + 2*K3 + K4) / 6;
        t = a + j*h;
    end
end

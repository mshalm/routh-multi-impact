function [W, w, Jn, JD] = RouthLCP(M, Jn, Jf, mu, v, lmax)

    [W, w, Jn, JD] = AnitescuLCP(M, Jn, Jf, mu, v);
    [m, ~] = size(Jn);
    I = eye(m);
    Z = zeros(m);

    W = [         Z, [-I Z Z Z];
         [I Z Z Z]',          W];

    w = [lmax;
            w];
end
function [W, w, Jn, JD] = AnitescuLCP(M, Jn, Jf, mu, v, active)
    
    if nargin < 6
        active = true(1, size(Jn,1));
    end
    
    Jn = Jn(active, :);
    Jf = Jf(active, :);

    mu = mu(active, active);
    
    [m, n] = size(Jn);

    JD = zeros(2 * m, n);
    JD(1:2:end, :) = Jf;
    JD(2:2:end, :) = -Jf;

    J = [Jn; JD];

    I = eye(m);
    E = repmat(I,2,1);
    E = reshape(E, [], size(E,1))';
    Z = zeros(m);
    z = zeros(m,1);

    W = [ J*(M\J'), [Z; E];
             [mu, -E'],     Z];
    w = [J*v(:,end);
         z];
end
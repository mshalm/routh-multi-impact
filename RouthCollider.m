function [v, l, terminated, err_max, i] = RouthCollider(M, Jn, Jf, mu, vm, h, N, p)
% initialize loop vars
v = vm;
[m, ~] = size(Jn);
l = zeros(3*m, 1);

% solver params
eps = 1e-8;
if nargin < 7
   N = inf; 
end
if nargin < 8
    p = rand(m, N);
end

i = 0;
terminated = true;
err_max = 0;
while min(Jn*v(:, end)) < -eps
    if i >= N
        terminated = false;
        break; 
    end
    lmax = h*p(:, i + 1);
    [W, w, Jn, JD] = RouthLCP(M, Jn, Jf, mu, v, lmax);
    
    sol = pathlcp(W, w);
    q = W * sol + w;
    err_max = max([err_max, -q', -sol', abs(sol .* q)']);
    J = [Jn; JD];
    dl = sol((m + 1):(4 * m));
    v(:, end + 1) = v(:, end) + M \ (J' * dl);
    l(:, end + 1) = dl;
    
    i = i + 1;
    
end

end
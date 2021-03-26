function [v, l] = AnitescuCollider(M, Jn, Jf, mu, vm, active, solver)

    % select default: all contacts actives
    if nargin < 6
        active = true(1, size(Jn, 1));
    end

    % select LCP solver
    if nargin < 7
       solver = @pathlcp;
    end

    % build LCP
    [W, w, Jn, JD] = AnitescuLCP(M, Jn, Jf, mu, vm, active);

    J = [Jn; JD];

    % solver for impulses
    n_active = sum(active > 0);
    sol = solver(W, w);
    l = sol(1:(3 * n_active),:);

    % update velocity
    v = vm + M \ (J' * l);
end
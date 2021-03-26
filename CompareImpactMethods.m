function [v_staged, v_equal, v_Routh] = ...
    CompareImpactMethods(fbd, mu, M_routh, N_routh, h, reduce)
    if nargin < 6
        reduce = true;
    end

    % construct impact terms
    [Jn, Jf] = fbd.contactJacobians();
    m = size(Jn, 1);
    M = fbd.massMatrix();
    vm = fbd.Velocity;

    [v_staged{1:m}] = deal(vm);
    v_active = 1:m;
    
    % impact termination colerance
    epsilon = eps * 1e10;
    
    % resolve impacts in alternating sequences
    N_staged = 10;
    for i = 1:N_staged
        still_running = false;
        for j = 1:m
            if min(Jn*v_staged{j}(:,end)) < - epsilon
                still_running = true;
                % do impact on 1 contact
                active = false(1, m);
                active(v_active(j)) = true;
                [v_staged{j}(:, end + 1), ~] = ...
                    AnitescuCollider(M, Jn, Jf, mu, ...
                    v_staged{j}(:, end), active);
                
                % cycle active contact
                v_active(j) = v_active(j) + 1;
                if v_active(j) > m
                    v_active(j) = 1;
                end
            end
        end
        if ~still_running
            break;
        end
    end
    
    % flag unique results via unique net inpulse
    [~, ~, ~, JD] = AnitescuLCP(M, Jn, Jf, mu, vm);
    J_ext = [Jn; JD];
    U = [J_ext * (M \ J_ext'), zeros(3 * m, m)];
    
    % resolve impacts concurrently
    v_equal = AnitescuCollider(M, Jn, Jf, mu, vm, true(1, m), ...
        @(W, w) LCPSolver_Remy(W, w, U));
    
    % resolve impacts via routh N_routh times
    
    % generate Routh force selectors
    if reduce
        dims = (m - 1) * N_routh;
    else
        dims = m * N_routh;
    end
    skip = 3 ^ min(N_routh, 20);
    leap = ceil(skip ^ 0.25);
    set = sobolset(dims, 'Skip', skip, 'Leap', leap);
    
    if reduce
        n_accepted = 0;
        k = zeros(M_routh, dims);
        while n_accepted < M_routh
            start = skip + n_accepted * (leap + 1);
            set.Skip = start;
            k_new = net(set, M_routh);
            reject = false(M_routh, 1);
            for i = 1:N_routh
                step_i = k_new(:, (i - 1) * (m - 1) + 1:(m - 1));
                reject = reject | sum(step_i, 2) > 1;
            end
            k_new = k_new(~reject, :);
            N_remain = M_routh - n_accepted;
            if size(k_new, 1) > N_remain
                k_new = k_new(1:N_remain, :);
            end
            k(n_accepted + (1:size(k_new, 1)), :) = k_new;
            n_accepted = n_accepted + size(k_new, 1);
        end
    else
        k = net(set, M_routh);
    end
    A = [eye(m-1); -ones(1, m - 1)];
    b = [zeros(m-1, 1); 1];
    
    v_Routh = cell(1, M_routh);
    v_Routh_finished = false(1, M_routh);
    routh_err = zeros(1, M_routh);
    parfor i = 1:M_routh
        k_i = k(i, :);
        if reduce
            k_i = reshape(k_i',[(m - 1) N_routh]);
            p = A * k_i + b;
        else
            p = reshape(k_i',[m N_routh]);
        end

        [v_Routh{i}, ~, v_Routh_finished(i), routh_err(i)] = ...
            RouthCollider(M, Jn, Jf, mu, vm, h, N_routh, p);

    end
    %{
    spmd
        inds = labindex:numlabs:M_routh;
        v_Routh_k = cell(1, length(inds));
        routh_err_k = zeros(1, length(inds));
        v_Routh_finished_k = false(1, length(inds));
        for i = 1:length(inds)
            k_i = k(inds(i), :);
            if reduce
                k_i = reshape(k_i',[(m - 1) N_routh]);
                p = A * k_i + b;
            else
                p = reshape(k_i',[m N_routh]);
            end

            [v_Routh_k{i}, ~, v_Routh_finished_k(i), routh_err_k(i)] = ...
                RouthCollider(M, Jn, Jf, mu, vm, h, N_routh, p);
            
        end
    
    end
    
    for i = 1:length(inds)
        v_Routh(inds{i}) = v_Routh_k{i};
        routh_err(inds{i}) = routh_err_k{i};
        v_Routh_finished(inds{i}) = v_Routh_finished_k{i};
    end
    %}
    TOL = 1e-6;
    accepted = v_Routh_finished & (routh_err <= TOL);
    v_Routh = v_Routh(accepted);
    fprintf("Routh Trajectories completed:\n max violation %.10e\n completion percentage %f \n accept percentage %f \n\n", ...
        max(routh_err), 100 * nnz(v_Routh_finished) / numel(v_Routh_finished), ...
        100 * nnz(accepted) / numel(accepted));
end


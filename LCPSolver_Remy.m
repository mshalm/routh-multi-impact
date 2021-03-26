function [x_res, y_res] = LCPSolver_Remy(A, b, U)
% LCPSolver is a naive solver for linear complimentary problems of the form
% y = A*x + b with x>0, y>0, and x*y = 0 
% This solver returns every solution by brute force.
%
% if U flag is set, only solutions corresponding to a unique U * x are
% recorded
    unique = nargin >= 3;
    if ~unique
        U = zeros(numel(b));
    end

    % Check input dimensions
    [i,j] = size(A);
    [n,k] = size(b);
    if k~=1
        error('b must be a column vector')
    end
    if i~=n || j~=n
        error('A should be a square matrix of length b')
    end
    x_res = zeros(n, 0);
    y_res = zeros(n, 0);
    x_tmp = nan(n, 2 ^ n);
    y_tmp = nan(n, 2 ^ n);
    % Check if input is an actual number:
    if all(all(isfinite(A)))
        % create matrix of permuations:
        combs = dec2bin(0:2^n-1) == '1';
        indices = 1:n;
        parfor i = 1:2^n
            %for i = 1:length(inds)
            % prepare output
            A_part = A(combs(i, :), combs(i, :));
            if rank(A_part)/nnz(combs(i, :))==1
                x = zeros(n,1);
                x_part = A_part\-b(combs(i, :));
                x(combs(i, :)) = x_part;
                y = A*x + b;
                y(combs(i, :))=0;
                if all(x>=-1e-8) && all(y>=-1e-8)
                    x_tmp(:, i) = x;
                    y_tmp(:, i) = y;
                end
            end
        end
        
        
        % filter
        success = ~any(isnan(x_tmp));
        x_tmp = x_tmp(:, success);
        y_tmp = y_tmp(:, success);
        
        % check unique
        for i=1:size(x_tmp, 2)
            x = x_tmp(:, i);
            y = y_tmp(:, i);
            diff = [vecnorm(U * (x_res - x)) inf];
            if ~unique || min(diff) > (eps * 1e6)
                x_res(:, end + 1) = x;
                y_res(:, end + 1) = y;
            end
        end
    end
    
end


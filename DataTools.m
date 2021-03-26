classdef DataTools
    
    methods (Static)
        function keeps = SparseScatter(data, N)
            % cols are datapoints
            if nargin < 2
               N = 100; 
            end
            
            if numel(N) == 1
                N = N * ones(size(data, 1), 1); 
            end
 
            min_d = min(data, [], 2);
            max_d = max(data, [], 2);
            stride = max_d - min_d;
            
            data = data - min_d;
            data = data ./ stride;
            data = data .* N;
            data = ceil(data);
            data = max(data, 1);
            data = min(data, N);
            
            
            ind_mat = false(1, prod(N));
            
            % 2d hack
            nz_ind = sub2ind(N(:)', data(1, :), data(2, :));
            ind_mat(nz_ind) = true;
            
            % filter
            keeps = zeros(1, prod(N));
            keeps(nz_ind) = 1:size(data, 2);
            keeps = keeps(:, ind_mat);
        end
        
        function sdata = LineInpaint(data, N)
            
            % calculate trajectory length
            N_orig = size(data, 2);
            
            % calculate number of points required for inpainting
            dx = diff(data, 1, 2);
            
            min_d = min(data, [], 2);
            max_d = max(data, [], 2);
            stride = max_d - min_d;
            delta = stride ./ N;
            
            N_pts = max(ceil(abs(dx ./ delta)));
            N_pts = max(N_pts, 1);
            t = [0 cumsum(N_pts)]';
            fdata = interp1(t, data', 0:sum(N_pts))';
            keeps = DataTools.SparseScatter(fdata, N);
            sdata = fdata(:, keeps);
        end
    end
end


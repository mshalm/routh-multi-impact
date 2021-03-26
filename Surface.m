classdef Surface < Shape

    properties
    end
    
    methods
        function obj = Surface(varargin)
            % occupes half-plane y <= Offset
            obj@Shape(varargin{:});
            
            % MAX ~ inf
            MAX = 1e2;
            obj.Vertices = [-MAX  MAX MAX -MAX;
                            -MAX -MAX   0    0];
        end
        
        function J = inertia(obj)
            J = 0;
        end
        
    end
    
end


classdef Point < Shape
    properties
        Inertia 
    end
    
    methods
        function obj = Point(J, varargin)
            obj@Shape(varargin{:});
            obj.Vertices = [0 0]';
            obj.Inertia = J;
        end
        
        function J = inertia(obj)
            J = obj.Inertia;
        end
        
        function params = collisionParams(~)
            params = [0 0 1];
        end
        
        function ppib = plotPointsInBase(obj, parent_tf)
            r = 0.02 * sqrt(obj.Mass);
            t = 0:0.01:(2*pi);
            points = r * [cos(t); sin(t)];
            ppib = obj.pointsInBase(points, parent_tf);
        end
        
    end
    
end


classdef Rod < Shape

    properties
        Length
    end
    
    methods
        function obj = Rod(l, varargin)
            obj@Shape(varargin{:});
            obj.Length = l;
            obj.Vertices = [-l l;
                             0 0]/2;
            obj.Hooks = {'l', 'r'};
        end
        
        function J = inertia(obj)
            J = obj.Mass * (obj.Length^2) / 12;
        end
        
        function params = collisionParams(obj)
            params = [obj.Length 0 1];
        end
        
        function ppib = plotPointsInBase(obj, parent_tf)
            verts = obj.Vertices;
            perp = [0 1]' * obj.Length /200;
            points = [verts - perp, fliplr(verts + perp)];
            ppib = obj.pointsInBase(points, parent_tf);
        end
        
    end
    
end


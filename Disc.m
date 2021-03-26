classdef Disc < Shape

    properties
        Radius
    end
    
    methods
        function obj = Disc(r, varargin)
            obj@Shape(varargin{:});
            obj.Radius = r;
            t = 0:0.01:2*pi;
            obj.Vertices = r*[cos(t); sin(t)];
        end
        
        function J = inertia(obj)
            J = obj.Mass * obj.Radius^2/2;
        end
        
        function params = collisionParams(obj)
            params = [obj.Radius 1];
        end
        
        function bb = boundingBox(obj, parent_tf)
            if nargin < 2
               parent_tf = eye(4); 
            end
            tform = parent_tf * obj.Transform;
            [R, p] = FreeBodyDiagram.tform2Rp(tform);
            bb = [p(1) - obj.Radius, p(1) + obj.Radius, ...
                  p(2) - obj.Radius, p(2) + obj.Radius];
        end
        
    end
    
end


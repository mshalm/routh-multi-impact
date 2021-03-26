classdef Box < Shape

    properties
        Width
        Height
    end
    
    methods
        function obj = Box(w, h, varargin)
            obj@Shape(varargin{:});
            obj.Width = w;
            obj.Height = h;
            obj.Vertices = [0 w w 0;
                            0 0 h h] - [w; h]/2;
            obj.Hooks = {'bl','br','tl','tr'};
        end
        
        function J = inertia(obj)
            J = obj.Mass * (obj.Width^2 + obj.Height^2) / 12;
        end
        
        function params = collisionParams(obj)
            params = [obj.Width obj.Height 1];
        end
        
    end
    
end


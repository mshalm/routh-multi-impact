classdef Rectangle < Shape

    properties
        height
        width
        m
    end
    
    methods
        function obj = Rectangle(h, w, m, varargin)
            obj@Shape(varargin{:});
            obj.height = h;
            obj.width = w;
            obj.m = m;
        end
        
        function v = verts(obj)
            h = obj.height;
            w = obj.width;
            v = [0 w w 0;
                 0 0 h h];
            v = v - [w; h]/2;
        end
        
        function m = intertia(obj)
            m = obj.mass() * (obj.width^2 + obj.height^2) / 12;
        end
        
        function m = mass(obj)
            m = obj.m;
        end
        
        function [obj, ax, pl] = push(obj, ax, c)
            [obj, ax, pl] = push@Shape(obj.verts(), ax, c);
        end
        
        function pp = render(obj, tf)
            if nargin < 2
               tf = Transform(); 
            end
            points = obj.location(tf * obj.verts());
            pgon = polyshape(points(1,:),points(2,:));
            pp = plot(pgon); 
        end
    end
end


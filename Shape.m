classdef Shape

    properties
        Transform
        Mass
        Vertices = zeros(2,0);
        PlotHandle
        Name
        ParentName
        Hooks = {};
        Color = [];
        Collision = true;
    end
    
    methods
        function obj = Shape(name, parent, mass, color, tf, collision)
            if nargin < 5
                tf = eye(4);
            end
            obj.Name = name;
            obj.ParentName = parent;
            obj.Mass = mass;
            obj.Transform = tf;
            if nargin >= 4 && numel(color) > 0
               obj.Color = PlotStyler.colorComponents(color); 
            end
            if nargin >= 6
                obj.Collision = collision;
            end
        end
        
        function J = inertia(~)
            J = 0;
        end
        
        function params = collisionParams(~)
            params = 0;
        end
        
        function vib = verticesInBase(obj, parent_tf)
            if nargin < 2
               parent_tf = eye(4);
            end
            vib = obj.pointsInBase(obj.Vertices, parent_tf);
        end
        
        function ppib = plotPointsInBase(obj, parent_tf)
            ppib = obj.verticesInBase(parent_tf);
        end
        
        function pib = pointsInBase(obj, pts, parent_tf)
            if isa(pts, 'char')
                pts = obj.getVertex(pts);
            end
            
            if size(pts,1) < 2
                pts = obj.Vertices(:,pts);
            end
            
            if nargin < 3
               parent_tf = eye(4);
            end
            tform = parent_tf * obj.Transform;
            [R, p] = FreeBodyDiagram.tform2Rp(tform);
            pib = R * pts + p;
        end
        
        function bb = boundingBox(obj, parent_tf)
            if nargin < 2
               parent_tf = eye(4); 
            end
            points = obj.plotPointsInBase(parent_tf);
            bb = [min(points(1,:)), max(points(1,:)), ...
                  min(points(2,:)), max(points(2,:))];
        end
        
        function obj = render(obj, parent_tf)
            if nargin < 2
               parent_tf = eye(4); 
            end
            points = obj.plotPointsInBase(parent_tf);
            pgon = polyshape(points(1,:),points(2,:));
            obj.PlotHandle = plot(pgon);
            if numel(obj.Color) > 0
                obj.PlotHandle.FaceColor = obj.Color;
            end
            
        end
        
        function idx = getVertex(obj, hook)
            idx = 0;
            for i=1:length(obj.Hooks)
                a = 0;
                if strcmp(hook, obj.Hooks{i})
                    idx = i;
                    break;
                end
            end
            if idx < 1
                error('unrecognized hook!');
            end
        end
    end
end


classdef FreeBodyDiagram
    %FREEBODYDIAGRAM
    
    properties
        Tree
        Shapes
        Configuration
        Velocity
        Styler
        PhiTolerance = 1e-6;
    end
    
    methods
        function obj = FreeBodyDiagram()
            obj.Tree = rigidBodyTree();
            obj.Tree.Gravity = [0 (-9.81) 0];
            obj.Tree.DataFormat = 'column';
            
            obj.Shapes = {};
            
            obj.Configuration = [];
            obj.Velocity = [];
        end
        
        function obj = home(obj)
            obj.Configuration = obj.Tree.homeConfiguration();
            obj.Velocity = 0 * obj.Configuration;
        end
        
        function M = massMatrix(obj)
            M = obj.Tree.massMatrix(obj.Configuration);
        end
        
        function M = forwardDynamics(obj)
            M = obj.Tree.forwardDynamics(obj.Configuration, obj.Velocity);
        end
        
        function obj = addFrame(obj, parent, name, jtype, varargin)
            new_frame = FreeBodyDiagram.zeroFrame(name);
            obj = obj.addBody(new_frame, parent, jtype, varargin{:});
            
            % new body added, reset config
            obj = obj.home();
        end
        
        function J = geometricJacobian(obj, framename)
            Jf = [0 0 0 1 0 0;
                  0 0 0 0 1 0;
                  0 0 1 0 0 0];
            J = Jf * obj.Tree.geometricJacobian(obj.Configuration, ...
                framename);
        end
        
        function tf = getTransform(obj, varargin)
            tf = obj.Tree.getTransform(obj.Configuration, varargin{:});
        end
        
        function J = pointJacobian(obj, shapename, points)
            points = obj.worldFrameOriginDisplacement(shapename, points);
            shape = obj.getShape(shapename);
            N = size(points, 2);
            Jf = repmat(eye(3), [N 1]);
            Jf(1:3:end,3) = -points(2,:)';
            Jf(2:3:end,3) = points(1,:)';
            J = Jf * obj.geometricJacobian(shape.ParentName);
        end
        
        function pts_world = worldFrameOriginDisplacement(obj, shapename, points)
            shape = obj.getShape(shapename);
            shape_tf = obj.getTransform(shape.ParentName);
            [~, origin] = FreeBodyDiagram.tform2Rp(shape_tf);
            pts_world = shape.pointsInBase(points, shape_tf) - origin;
        end
        
        function obj = labelPoint(obj, shapename, local_pt, varargin)
            shape = obj.getShape(shapename);
            shape_tf = obj.getTransform(shape.ParentName);
            point = shape.pointsInBase(local_pt, shape_tf);
            obj.Styler.plotPoint(point, varargin{:});
        end
        
        function obj = labelSpin(obj, shapename, local_pt, size, spin, varargin)
            shape = obj.getShape(shapename);
            shape_tf = obj.getTransform(shape.ParentName);
            point = shape.pointsInBase(local_pt, shape_tf);
            obj.Styler.plotSpin(point, size, spin, varargin{:});
        end
        
        function obj = labelArrow(obj, shapename, local_pt, arr, varargin)
            shape = obj.getShape(shapename);
            shape_tf = obj.getTransform(shape.ParentName);
            point = shape.pointsInBase(local_pt, shape_tf);
            obj.Styler.plotArrow(point, arr, varargin{:});
        end
        
        function obj = labelVelocity(obj, shapename, local_pt, varargin)
            V = [eye(2) zeros(2,1)] * ...
                obj.pointJacobian(shapename, local_pt) * ...
                obj.Velocity;
            obj.labelArrow(shapename, local_pt, V, varargin{:});
        end
        
        function obj = labelAngularVelocity(obj, shapename, local_pt, size, varargin)
            omega = [0 0 1] * ...
                obj.pointJacobian(shapename, local_pt) * ...
                obj.Velocity;
            obj.labelSpin(shapename, local_pt, size, omega, varargin{:});
        end
        
        function [pts1, pts2, normals, phi] = pairContacts(obj, b1, b2)
            if b1.Collision && b2.Collision
                if isa(b1, 'Surface') && isa(b2, 'Box')
                    [pts1, pts2, normals, phi] = ...
                        wallBoxContacts(obj, b1, b2);
                elseif isa(b1, 'Box') && isa(b2, 'Surface')
                    [pts2, pts1, normals, phi] = ...
                        wallBoxContacts(obj, b2, b1);
                    normals = -normals;
                elseif isa(b1, 'Surface') && isa(b2, 'Rod')
                    [pts1, pts2, normals, phi] = ...
                        wallRodContacts(obj, b1, b2);
                elseif isa(b1, 'Rod') && isa(b2, 'Surface')
                    [pts2, pts1, normals, phi] = ...
                        wallRodContacts(obj, b2, b1);
                    normals = -normals;
                elseif isa(b1, 'Surface') && isa(b2, 'Point')
                    [pts1, pts2, normals, phi] = ...
                        wallPointContacts(obj, b1, b2);
                elseif isa(b1, 'Point') && isa(b2, 'Surface')
                    [pts2, pts1, normals, phi] = ...
                        wallPointContacts(obj, b2, b1);
                    normals = -normals;
                elseif isa(b1, 'Surface') && isa(b2, 'Disc')
                    [pts1, pts2, normals, phi] = ...
                        wallDiscContacts(obj, b1, b2);
                elseif isa(b1, 'Disc') && isa(b2, 'Surface')
                    [pts2, pts1, normals, phi] = ...
                        wallDiscContacts(obj, b2, b1);
                    normals = -normals;
                elseif isa(b1, 'Disc') && isa(b2, 'Disc')
                    [pts1, pts2, normals, phi] = ...
                        discDiscContacts(obj, b1, b2);
                else
                    pts1 = zeros(2, 0);
                    pts2 = zeros(2, 0);
                    normals = zeros(2, 0);
                    phi = zeros(1,0);
                end
            else
                pts1 = zeros(2, 0);
                pts2 = zeros(2, 0);
                normals = zeros(2, 0);
                phi = zeros(1,0);
            end
        end
        
        function [wall_pts, rod_pts, normals, phi] = wallRodContacts(obj, wall, rod)
            [wall_pts, rod_pts, normals, phi] = ...
                obj.wallPtsContacts(wall, rod, rod.Vertices);
        end
        
        function [wall_pts, pt_pts, normals, phi] = wallPointContacts(obj, wall, pt)
            [wall_pts, pt_pts, normals, phi] = ...
                obj.wallPtsContacts(wall, pt, [0 0]');
        end
        
        function [wall_pts, box_pts, normals, phi] = wallBoxContacts(obj, wall, box)
            [wall_pts, box_pts, normals, phi] = ...
                obj.wallPtsContacts(wall, box, box.Vertices);
        end
        
        function [wall_pts, shape_pts, normals, phi] = wallPtsContacts(obj, wall, shape, shape_pts)
            shape_tf = obj.getTransform(shape.ParentName) * shape.Transform;
            wall_tf = obj.getTransform(wall.ParentName) * wall.Transform;
            box_to_wall = FreeBodyDiagram.tforminv(wall_tf) * shape_tf;
            [R, p] = FreeBodyDiagram.tform2Rp(box_to_wall);
            wall_pts = R * shape_pts + p;
            phi = wall_pts(2,:);
            % project onto surface
            wall_pts(2,:) = 0;
            % need WORLD FRAME normals
            [R, ~] = FreeBodyDiagram.tform2Rp(wall_tf);
            normals = R * [zeros(1,4); ones(1,4)];
        end
        
        function [d1_pts, d2_pts, normals, phi] = discDiscContacts(obj, d1, d2)
            d1_tf = obj.getTransform(d1.ParentName) * d1.Transform;
            d2_tf = obj.getTransform(d2.ParentName) * d2.Transform;
            d2_to_d1 = FreeBodyDiagram.tforminv(d1_tf) * d2_tf;
            [R, p] = FreeBodyDiagram.tform2Rp(d2_to_d1);
            
            normal = p / norm(p);
            phi = norm(p) - d1.Radius - d2.Radius;
            d1_pts = d1.Radius * normal;
            d2_pts = - d2.Radius * R' * normal;
            
            % need WORLD FRAME normals
            [R, ~] = FreeBodyDiagram.tform2Rp(d1_tf);
            normals = R * normal;
        end
        
        function [wall_pts, disc_pts, normals, phi] = wallDiscContacts(obj, wall, disc)
            disc_tf = disc.Transform * obj.getTransform(disc.ParentName);
            wall_tf = wall.Transform * obj.getTransform(wall.ParentName);
            disc_to_wall = FreeBodyDiagram.tforminv(wall_tf) * disc_tf;
            [R, p] = FreeBodyDiagram.tform2Rp(disc_to_wall);
            disc_pts = R' * [0 -disc.Radius]';
            wall_pts = p;
            phi = wall_pts(2,:) - disc.Radius;
            % project onto surface
            wall_pts(2,:) = 0;
            % need WORLD FRAME normals
            [R, ~] = FreeBodyDiagram.tform2Rp(wall_tf);
            normals = R * [zeros(1,4); ones(1,4)];
        end
        
        function [Jn, Jf] = contactJacobians(obj, varargin)
            [Jn, Jf] = obj.contactTerms(varargin{:});
        end
        
        function [Jn, Jf, pts, bodies, phi_all, bias_all] = contactTerms(obj, active)
            if nargin < 2
                active = true;
            end
            V = numel(obj.Velocity);
            Jn = zeros(0,V);
            Jf = zeros(0,V);
            phi_all = zeros(0,1);
            bias_all = zeros(0,2);
            pts = cell(1, 0);
            bodies = cell(1, 0);
            for i = 1:length(obj.Shapes)
                for j = i:length(obj.Shapes)
                    b1 = obj.Shapes{i};
                    b2 = obj.Shapes{j};
                    
                    % no intrabody contact
                    if ~strcmp(b1.ParentName, b2.ParentName)
                        [p1, p2, n, phi] = obj.pairContacts(b1, b2);
                        if active
                            ac = phi <= obj.PhiTolerance;
                            phi = phi(ac);
                            p1 = p1(:,ac);
                            p2 = p2(:,ac);
                            n = n(:,ac);
                        end
                        
                        if numel(phi) > 0
                            for k = 1:length(phi)
                                if isa(b1, 'Surface')
                                    pts{end + 1} = p2(:, k);
                                    bodies{end + 1} = b2.Name;
                                else
                                    pts{end + 1} = p1(:, k);
                                    bodies{end + 1} = b1.Name;
                                end
                            end
                            J1 = obj.pointJacobian(b1.Name, p1);
                            J2 = obj.pointJacobian(b2.Name, p2);
                            [Jn12, Jf12] = FreeBodyDiagram. ...
                                pointToContactJacobians(n, J1, J2);
                            Jn = [Jn; Jn12];
                            Jf = [Jf; Jf12];
                            phi_all = [phi_all; phi(:)];
                            
                            % calculate bias term (dJ/dt) * v
                            % in 2d, bias is simply -pts * omega^2
                            % need to put in world frame coords
                            p1_w = obj.worldFrameOriginDisplacement(b1.Name, p1);
                            v1_spatial = J1 * obj.Velocity;
                            omega1 = v1_spatial(3:3:end);
                            bias1 = -(omega1 .^ 2)' .* p1_w;
                            
                            p2_w = obj.worldFrameOriginDisplacement(b2.Name, p2);
                            v2_spatial = J2 * obj.Velocity;
                            omega2 = v2_spatial(3:3:end);
                            bias2 = -(omega2 .^ 2)' .* p2_w;
                            
                            bias_world = bias2 - bias1;
                            bias = obj.pointToContactBias(n, bias_world);
                            bias_all = [bias_all; bias'];
                        end
                    end
                end
            end
        end
        
        function obj = addBox(obj, parent, name, width, height, mass, varargin)
            params = [width, height];
            obj = obj.addItem(parent, name, 'box', params, mass, varargin{:});
        end
        
        function obj = addDisc(obj, parent, name, radius, mass, varargin)
            obj = obj.addItem(parent, name, 'disc', radius, mass, varargin{:});
        end
        
        function obj = addRod(obj, parent, name, length, mass, varargin)
            obj = obj.addItem(parent, name, 'rod', length, mass, varargin{:});
        end
        
        function obj = addPoint(obj, parent, name, inertia, mass, varargin)
            obj = obj.addItem(parent, name, 'point', inertia, mass, varargin{:});
        end
        
        function obj = addWall(obj, name, normal, offset, color)
            if nargin < 3
                normal = [0 1]';
            end
            if nargin < 4
                offset = 0;
            end
            if nargin < 5
                color = [];
            end
            normal = normal(:)/norm(normal);
            R = [ normal(2) normal(1);
                 -normal(1) normal(2)];
            p = normal*offset;
            tform = FreeBodyDiagram.Rp2tform(R, p);
            obj = obj.addItem('base', name, 'surface', [], 1, color, tform);
        end
        
        function obj = addItem(obj, parent, name, type, params, ...
                mass, color, tform, collision)
            if nargin < 7
                color = [];
            end
            if nargin < 8
                tform = eye(4);
            end
            if nargin < 9
                collision = true;
            end
            
            if strcmp(parent,'base')
                frame = obj.Tree.Base;
            else
                frame = obj.Tree.getBody(parent);
            end
            
            switch type
                case 'box'
                    item_type = 'box';
                    w = params(1);
                    h = params(2);
                    item = Box(w, h, name, parent, mass, color, ...
                        tform, collision);
                    
                case 'disc'
                    item_type = 'cylinder';
                    r = params;
                    item = Disc(r, name, parent, mass, color, ...
                        tform, collision);
                    
                case 'rod'
                    item_type = 'box';
                    l = params;
                    item = Rod(l, name, parent, mass, color, tform, ...
                        collision);
                    
                case 'point'
                    item_type = 'box';
                    J = params;
                    item = Point(J, name, parent, mass, color, tform, ...
                        collision);
                    
                case 'surface'
                    item = Surface(name, parent, mass, color, tform, ...
                        collision);
                    
                otherwise
                    error('unknown item type!')
            end
            
            switch type
                case {'box', 'disc', 'rod', 'point'}
                    addCollision(frame, item_type, ...
                        item.collisionParams(), tform);
                    frame = FreeBodyDiagram.InertialAdd(frame, ...
                                mass, item.inertia(), tform);
                    replaceBody(obj.Tree, parent, frame);
            end

            obj.Shapes{end + 1} = item;
        end
        
        function shape = getShape(obj, shapename)
            found = false;
            for i = 1:length(obj.Shapes)
                shape = obj.Shapes{i};
                if strcmp(shapename, shape.Name)
                    found = true;
                    break;
                end
            end
            if ~found
               error('no such shape found!') 
            end
        end
        
        function bb = getBoundingBox(obj, shapename)
            shape = obj.getShape(shapename);
            parent_tf = obj.getTransform(shape.ParentName);
            bb = shape.boundingBox(parent_tf);
        end
        
        function obj = view(obj, label_callback, stage, limits)
            if nargin < 2
               label_callback = @(x, s) x; 
            end
            if nargin < 3
                stage = 0;
            end
            obj.Styler = PlotStyler();
            if nargin > 3
                limits(1:2) = PlotStyler.padAxis(limits(1:2), 0.05, 0.02);
                limits(3:4) = PlotStyler.padAxis(limits(3:4), 0.05, 0.02);
                dy = limits(4) - limits(3);
                dx = limits(2) - limits(1);
                aspect = dx / dy;
                
                % prevent absurd aspect ratios
                AMAX = 2;
                aspect = min(aspect, AMAX);
                aspect = max(aspect, inv(AMAX));
                
                obj.Styler.aspect = aspect;
                if aspect > 1
                    obj.Styler.height = obj.Styler.height / aspect;
                end
            end
            InSet = get(gca, 'TightInset');
            set(gca, 'Position', ...
                [InSet(1:2), 1-InSet(1)-InSet(3), 1-InSet(2)-InSet(4)]);
            bbox = zeros(0,4);
            for i = 1:length(obj.Shapes)
                shape = obj.Shapes{i};
                if ~isa(shape,'Surface')
                    parent_tf = obj.getTransform(shape.ParentName);
                    obj.Shapes{i} = shape.render(parent_tf);
                    hold on;
                    bbox = [bbox; shape.boundingBox(parent_tf)];
                end
            end
            obj = label_callback(obj, stage);
            obj.Styler.axisStyle('equal off tight manual');
            for i = 1:length(obj.Shapes)
                shape = obj.Shapes{i};
                if isa(shape,'Surface')
                    parent_tf = obj.getTransform(shape.ParentName);
                    obj.Shapes{i} = shape.render(parent_tf);
                    hold on;
                    uistack(obj.Shapes{i}.PlotHandle, 'bottom');
                end
            end
            if nargin > 3
                xlim(limits(1:2));
                ylim(limits(3:4));
            end
            obj.Styler.asFreeBodyDiagram();
        end
        
        function obj = addBody(obj, body, parent, type, tform, axis)
            jname = FreeBodyDiagram.jointOf(body.Name);
            if nargin < 5
                tform = eye(4);
            end
            if nargin < 6
                axis = [0 0 1];
            end
            switch type
                case {'revolute', 'fixed', 'prismatic'}
                case 'floating'
                    body_xt = [body.Name '_xt'];
                    body_yt = [body.Name '_yt'];
                    obj = obj.addFrame(parent, body_xt, 'prismatic', ...
                        tform, [1 0 0]) ...
                        .addFrame(body_xt, body_yt, 'prismatic', ...
                        eye(4), [0 1 0]);
                    type = 'revolute';
                    parent = body_yt;
                    tform = eye(4);
                    axis = [0 0 1];
            end
            jnt = rigidBodyJoint(jname, type);
            jnt.PositionLimits = [-inf, inf];
            jnt.JointAxis = axis;
            setFixedTransform(jnt, tform);
            body.Joint = jnt;
            addBody(obj.Tree, body, parent);
        end
    end
    
    methods (Static)
        function body = zeroFrame(name)
            body = rigidBody(name);
            body.Mass = 0;
            body.Inertia = zeros(1,6);
            body.CenterOfMass = zeros(1,3);
        end
        
        function js = jointOf(name)
           js = [name, '_jt']; 
        end
         
        function body = InertialAdd(body, m, I, tf)
            % assume uniaxial
            c = [tf(1:3, 4)'];
            
            m_ = m + body.Mass;
            if m_ > 0
                c_ = (m*c + body.Mass*body.CenterOfMass)/m_;
            else
                c_ = body.CenterOfMass;
            end
            I_ = body.Inertia(3) + I + m*(c*c');
            body.Mass = m_;
            body.CenterOfMass = c_;
            body.Inertia(3) = I_;
        end
        
        function tf = T(pos, rot)
           tf = eye(4);
           tf(1:2, 1:2) = FreeBodyDiagram.R(rot);
           tf(1:2, 4) = pos;
        end
        
        function [R, p] = tform2Rp(tf)
           R = tf(1:2, 1:2);
           p = tf(1:2, 4);
        end
        
        function rot = tform2rot(tf)
           rot = tform2axang(tf)*[0 0 0 1]';
        end
        
        function tf = Rp2tform(R, p)
           tf = trvec2tform([p' 0]) * rotm2tform([R zeros(2,1); 0 0 1]);
        end
        
        function tf = tforminv(tf)
           [R, p] = FreeBodyDiagram.tform2Rp(tf);
           Ri = R';
           pi = -Ri*p;
           tf = FreeBodyDiagram.Rp2tform(Ri, pi);
        end
        
        function rmat = R(rot)
           rmat = [cos(rot), -sin(rot);
                   sin(rot),  cos(rot)];
        end
        
        function [Jn, Jf] = pointToContactJacobians(n, J1, J2)
            N = size(n, 2);
            Q = size(J1, 2);
            Jn = zeros(N, Q);
            Jf = zeros(N, Q);
            Jrel = J2 - J1;
            for i=1:N
               T = FreeBodyDiagram.contactFrameProjection(n(:,i));
               Jrel_i = Jrel((1:2) + 3*(i-1), :);
               Jn(i, :) = T(1,:)*Jrel_i; 
               Jf(i, :) = T(2,:)*Jrel_i; 
            end
        end
        
        function bias = pointToContactBias(n, bias_world)
            N = size(n, 2);
            bias = zeros(2, N);
            bias = bias_world;
            % TODO: vectorize
            for i=1:N
                T = FreeBodyDiagram.contactFrameProjection(n(:,i));
                bias(:, i) = T * bias_world(:, i);
            end
        end
        
        function T = contactFrameProjection(n)
            T = [n';
                 n(2) n(1)];
        end
        
    end
end


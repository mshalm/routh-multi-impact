%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script to numerically compute the collision outcomes of a model of the  % 
% bipedal robot RAMone                                                    %
%                                                                         %
%   C. David Remy cdremy@umich.edu                                        %
%   Matlab R2012b                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Init everything
% Clear memory, command window, and close figures
clear all
close all
clc
%% Define parameters
m1       = 7.9026;            % [kg] mass of the main body
m2       = 0.7887;            % [kg] mass of the upper leg segments
m3       = 0.510;             % [kg] mass of the lower leg segments
j1       = 0.08;              % [kg*m^2] inertia of the main body
j2       = 0.00221;           % [kg*m^2] inertia of the upper leg segments
j3       = 0.00652;           % [kg*m^2] inertia of the lower leg segments
s1       = 0.137675;          % [m] Distance from the hip axis to pitch axis
s2       = 0.019399;          % [m] distance between hip joints and CoG of the upper leg segments
s3       = 0.073265; % 0.2385 - [m] distance between foot points and CoG of the lower leg segments
l2       = 0.2;               % [m] length of the upper leg segments
l3       = 0.2385;            % [m] length of the lower leg segments
rFoot    = 0.0563/2;          % [m] foot radius

mu = 1e5; % [.] coefficient of friction (~infinity)

DANG = 1;

theta_min     = 0; % [deg]
theta_max     = 45; % [deg]
theta_nums    = theta_min:DANG:theta_max;

dalph     = 0; % [deg]
dalph_max     = 45; % [deg]
dalph_nums    = dalph_min:DANG:dalph_max;

gamma_min     = 0; % [deg]
gamma_max     = 90;
gamma_nums    = gamma_min:DANG:gamma_max;

alpha_min = 0;
alpha_max_max = 75;
%kneeAngle = -30*pi/180;%-90*pi/180;  % Both knees have the same angle
%  % Main body pitch
% All other generalized coordinates follow from this and from the inner leg
% angle phi which is defined as going from vertically downwards to the line
% 'Hip to Feet'.  Furthermore, the back (left) leg is at [0;0];
% Generalized velocities:
phi_tr_dot_num = 1;
% All joint velocities (dalpha and dbeta) are 0 at impact.  Motion is
% created purely by rotation about the back (left) leg with phi_tr_dot_num


%%  Numerical evaluation of the LCP formulation:
% Prepare output:
max_diff = zeros(0, 1 + 3);
for i = 1:length(theta_nums)
    for j = 1:length(gamma_nums)
        % Compute generalized coordinates:
        theta_num = theta_nums(i)*pi/180;
        gamma_num = gamma_nums(j)*pi/180;
        % compute
        if theta_num <= 1e-6
            alpha_max = alpha_max_max;
        else 
            sin_alpha_max = l3 * (1 - sin(gamma_num)) / (2 * l2 * sin(theta_num));
            if sin_alpha_max >= sind(alpha_max_max)
                alpha_max = alpha_max_max;
            else
            
                alpha_max = asind(sin_alpha_max);
                if norm(alpha_max - alpha_max') > 1e-6
                    qq = 0;
                end
                alpha_max = floor(alpha_max / DANG) * DANG;
            end
        end
        
        
        alpha_nums = alpha_min:DANG:alpha_max;
        for k = 1:length(alpha_nums)
            alpha_num = alpha_nums(k)*pi/180;
            alpha_tr = alpha_num;
            alpha_le = -alpha_num;
            theta = theta_num;
            beta_le = - alpha_le - theta_num + gamma_num - pi/2;
            
            hip_ht = l3 * sin(gamma_num) + l2 * cos(theta_num + alpha_le);
            tr_knee_ht = hip_ht - l2 * cos(theta_num + alpha_tr);
            % tr_knee_ht = (l3 + 2 * l2 * sin(alpha_num)) * sin(gamma_num);
            
            gamma_tr = asin(tr_knee_ht/l3);
            beta_tr = -alpha_tr - theta_num + gamma_tr - pi/2;
            % Shift MB, such that trailing (left) foot is at (0,0) (Note foot
            % radius places the foot center above ground):
            x = 0;
            y = s1 * cos(theta_num) + hip_ht + rFoot;

            % Define generalized state at collision:
            q = [x; y; theta_num; alpha_tr; beta_tr; alpha_le; beta_le];
            %y - s1 * cos(theta_num) - l2 * cos(theta_num + alpha_le) ...
            %    - l3*cos(theta_num + alpha_le + beta_le) - rFoot
            %y - s1 * cos(theta_num) - l2 * cos(theta_num + alpha_tr) ...
            %    - l3*cos(theta_num + alpha_tr + beta_tr) - rFoot
            % Compute Mass Matrix and Contact Jacobian:
            [M, J] = RAMoneMassMatrixJacobian(q, m1, m2, m3, j1, j2, j3, s1, s2, s3, l2, l3, rFoot);
            % Define generalized velocities before the collision (such that the
            % trailing (left) foot is at rest):
            q_dot_MINUS = phi_tr_dot_num*[-J(1,3); -J(2,3); 1; 0; 0; 0; 0];
            % Compute pre-impact contact space velocities:
            c_dot_MINUS = J*q_dot_MINUS;

            % Contact Dynamics for LCP formulation:
            J_ext = [+J(1,:);
                -J(1,:);
                J(2,:);
                +J(3,:);
                -J(3,:);
                J(4,:)];
            % With this, we compute an extended inverse of the contact space mass
            % matrix:
            M_lambda_inv_ext = J_ext*(M\J_ext.');
            % And we can setup the components of the LCP
            A_num = [M_lambda_inv_ext,[1;0;0;0;0;0],[0;1;0;0;0;0],[0;0;0;1;0;0],[0;0;0;0;1;0];
                -1,  0, mu,  0,  0,  0, 0, 0, 0, 0;
                0, -1, mu,  0,  0,  0, 0, 0, 0, 0;
                0,  0,  0, -1,  0, mu, 0, 0, 0, 0;
                0,  0,  0,  0, -1, mu, 0, 0, 0, 0];
            b_num = [+c_dot_MINUS(1);
                -c_dot_MINUS(1);
                c_dot_MINUS(2);
                +c_dot_MINUS(3);
                -c_dot_MINUS(3);
                c_dot_MINUS(4);
                0;
                0;
                0;
                0];
            c_mat = [1 -1 0 0  0 0;
                     0  0 1 0  0 0;
                     0  0 0 1 -1 0;
                     0  0 0 0  0 1];
            
            Jn = [J(4,:); J(2,:)];
            Jf = [J(3,:); J(1,:)];
            [A_num, b_num, ~, JD] = AnitescuLCP(M, Jn, Jf, mu*eye(2), q_dot_MINUS);
            J_ext = [Jn; JD];
            M_lambda_inv_ext = J_ext*(M\J_ext.');
            U = [M_lambda_inv_ext, zeros(6,2)];
            % Solve LCP (this naive solver just tries all combinations and returns
            % 1024 results which are NaN if they are infeasible)
            %[x_num, y_num] = LCPSolver_IJRR(A_num, b_num);
            %[x_num, y_num] = LCPSolver_Remy(A_num, b_num, U);
            %v_plus = q_dot_MINUS + M \ J_ext' * x_num(1:6, :);
            v_p = AnitescuCollider(M, Jn, Jf, mu*eye(2), q_dot_MINUS, true(1,2), ...
                @(W, w) LCPSolver_Remy(W, w, U));
            % LCPSolver_IJRR(A_num, b_num);
            % check for unique post impact
            %x_num = x_num(:,~any(isnan(y_num)) & ~any(isnan(x_num)));
            dc_dot = J_ext * v_p;%M_lambda_inv_ext * x_num(1:6, :);
            %diffs = norm(c_dot(:,2:end)-c_dot(:,1), 1);
            diffs = norm(dc_dot-dc_dot(:,1), 1);
            m = max(diffs);
            if m > 1e-2
                max_new = [m, rad2deg([theta_num, beta_le, alpha_tr])]
                max_diff = [max_diff; max_new];
                if m > 0.03
                    mqqq = 0;
                end
            end
        end
    end
end
% Plot all 1024 solutions:
% Make an empty call to CreateVelLambdaFigure_IJRR, to set up the axis and labels 
%CreateVelLambdaFigure_IJRR(6, 'Combined Solutions RAMone Model', phi_nums, NaN(4,length(phi_nums)), NaN(4,length(phi_nums)));
% Plot post impact velocities

%{

subplot(211)
plot(phi_nums, c_dot_PLUS_tr_x,'m');
plot(phi_nums, c_dot_PLUS_tr_y,'r');
plot(phi_nums, c_dot_PLUS_le_x,'c');
plot(phi_nums, c_dot_PLUS_le_y,'b');
axis tight
% Plot impulses
subplot(212)
plot(phi_nums, Lambda_tr_x,'m');
plot(phi_nums, Lambda_tr_y,'r');
plot(phi_nums, Lambda_le_x,'c');
plot(phi_nums, Lambda_le_y,'b');
axis tight


%% Compute the three outcomes for phi_num = 0.5;
phi_num = 0.5;
lVirtual = sqrt(l2^2 + l3^2 - 2*l2*l3*cos(pi+kneeAngle));
% Compute offset angle of upper leg segment from this virtual leg:
angleOffset = asin(l3/lVirtual*sin(pi+kneeAngle));
% Compute hip angles:
alphaL = angleOffset+phi_num-pitchMB;
alphaR = angleOffset-phi_num-pitchMB;
% Shift MB, such that trailing (left) foot is at (0,0) (Note foot
% radius places the foot center above ground):
x = -s1*sin(pitchMB) - l2*sin(alphaL+pitchMB) - l3*sin(kneeAngle+alphaL+pitchMB);
y = +s1*cos(pitchMB) + l2*cos(alphaL+pitchMB) + l3*cos(kneeAngle+alphaL+pitchMB) + rFoot;

% Define generalized state at collision:
q = [x; y; pitchMB; alphaL; kneeAngle; alphaR; kneeAngle];
% Compute Mass Matrix and Contact Jacobian:
[M, J] = RAMoneMassMatrixJacobian(q, m1, m2, m3, j1, j2, j3, s1, s2, s3, l2, l3, rFoot);
% Define generalized velocities before the collision (such that the
% trailing (left) foot is at rest):
q_dot_MINUS = phi_tr_dot_num*[-J(1,3); -J(2,3); 1; 0; 0; 0; 0];
% Compute pre-impact contact space velocities:
c_dot_MINUS = J*q_dot_MINUS;

% Compute M_lambda
M_lambda = inv(J*(M\J'));
disp('Contact space mass matrix:');
disp('M_lambda = ');
disp(M_lambda);

%% Partitioning of the contact space mass matrix:
% Partition the contact space mass matrix into seperate components for the
% trailing and leading legs, as well as a coupling matrix:
M_tr = M_lambda(1:2,1:2);
M_le = M_lambda(3:4,3:4);
M_cp = M_lambda(1:2,3:4);

% Partition the impact velocities, as well:
c_tr_dot_MINUS = c_dot_MINUS(1:2);
c_le_dot_MINUS = c_dot_MINUS(3:4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Analytical computation of the three basic contact cases:                %
% Case 1: Collision at leading foot only, trailing foot leaves the ground %
% Case 2: Collisions at leading and trailing foot, trailing foot stops    %
% Case 3: Sliding with an infinite coefficient of friction of mu          %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Case 1: Collision at swing foot only, stance foot leaves the ground:
c_tr_dot_PLUS_1 = M_tr\(M_cp*c_le_dot_MINUS);
c_le_dot_PLUS_1 = [0;0];
Lambda_tr_1 = [0;0];
Lambda_le_1 = M_cp.'*(c_tr_dot_PLUS_1 - c_tr_dot_MINUS) + ...
              M_le*(c_le_dot_PLUS_1 - c_le_dot_MINUS);
disp('Case 1: Trailing Foot Velocity:')
disp(c_tr_dot_PLUS_1)
disp('Case 1: Trailing Foot Impulse:')
disp(Lambda_tr_1)
          
%% Case 2: Collisions at stance and swing foot, stance foot stops:
c_tr_dot_PLUS_2 = [0;0];
c_le_dot_PLUS_2 = [0;0];
Lambda_tr_2 = M_tr*(c_tr_dot_PLUS_2 - c_tr_dot_MINUS) + ...
              M_cp*(c_le_dot_PLUS_2 - c_le_dot_MINUS);
Lambda_le_2 = M_cp.'*(c_tr_dot_PLUS_2 - c_tr_dot_MINUS) + ...
              M_le*(c_le_dot_PLUS_2 - c_le_dot_MINUS);
disp('Case 2: Trailing Foot Velocity:')
disp(c_tr_dot_PLUS_2)
disp('Case 2: Trailing Foot Impulse:')
disp(Lambda_tr_2)

%% Case 3: Limiting case, sliding to the left with an infinite coefficient of friction:
c_tr_x_dot_PLUS = 1/M_tr(2,1)*(M_cp(2,:)*c_le_dot_MINUS);
c_tr_dot_PLUS_3 = [c_tr_x_dot_PLUS;0];
c_le_dot_PLUS_3 = [0;0];
Lambda_tr_3 = M_tr*(c_tr_dot_PLUS_3 - c_tr_dot_MINUS) + ...
              M_cp*(c_le_dot_PLUS_3 - c_le_dot_MINUS);
Lambda_le_3 = M_cp.'*(c_tr_dot_PLUS_3 - c_tr_dot_MINUS) + ...
              M_le*(c_le_dot_PLUS_3 - c_le_dot_MINUS);
disp('Case 3: Trailing Foot Velocity:')
disp(c_tr_dot_PLUS_3)
disp('Case 3: Trailing Foot Impulse:')
disp(Lambda_tr_3)


%}




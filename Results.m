function Results(only_plot)
    if nargin == 0
        only_plot = false(1,5);
    end
    clf;
    close all;
    BoxWall(only_plot(1));
    Phone(only_plot(2));
    CompareCompliantContact(true);
    CompassGait(only_plot(3));
    RAMone(only_plot(4));
    BallStack(only_plot(5));
end

function BoxWall(varargin)

    % dimensions
    w = 1;
    
    % mass
    m = 1;
    
    % friction
    mu = 1;
    mu_mat = diag([mu mu]);
    
    % velocity
    v0 = 1;
    
    % sim params
    h = 2;
    N_Routh = round(10 / h);
    M_Routh = 2 ^ 18;
    

    % construct system
    fbd = FreeBodyDiagram();
    fbd = fbd.addFrame('base', 'object', 'floating');
    fbd = fbd.addBox('object', 'box', w, w, m, 'y');
    fbd = fbd.addWall('floor', [0 1], 0, 'l2');
    fbd = fbd.addWall('right', [-1 0], 0, 'l2');
    
    % construct initial condition
    theta = deg2rad(10);
    fbd.Configuration = [0 0 (pi/2 - theta)]';
    
    % move box to avoid penetration
    bb = fbd.getBoundingBox('box');
    fbd.Configuration(1) = -bb(2);
    fbd.Configuration(2) = -bb(3);
    fbd.Velocity = [v0 0 0]';
    
    % set up labelling callback
    label_callback = @(f, s) f.labelVelocity(...
        'box', [0 0]', v_of_stage(s), 'r');
    
    % run experiment
    TwoContactExperiment(fbd, mu_mat, M_Routh, N_Routh, h, ...
        label_callback, varargin{:});
end

function CompassGait(varargin)
    % dimensions
    % leg length
    l = 1;
    
    % mass position
    s_parallel = l/2;
    s_perpendicular = 0;
    mass_pos = [s_perpendicular -s_parallel 0];
    
    % leg mass
    m = 1;
    
    % collision angle
    phi = deg2rad(78);
    
    % initial condition components
    x0 = -l * sin(phi);
    y0 =  l * cos(phi);
    
    dphi0 = 2 ^ (-2);
    dx0 = -l * cos(phi) * dphi0;
    dy0 = -l * sin(phi) * dphi0;
    
    
    % friction
    mu = 5;
    mu_mat = diag([mu mu]);
    
    % sim params
    h = 1 * dphi0; %.25
    N_Routh = 5;
    M_Routh = 2 ^ 20; %21
    
    % construct system
    fbd = FreeBodyDiagram();
    
    % pelvis
    fbd = fbd.addFrame('base', 'pelvis_x', 'prismatic', eye(4), [1 0 0]);
    fbd = fbd.addFrame('pelvis_x', 'pelvis_y', 'prismatic', eye(4), ...
        [0 1 0]);
    
    % legs
    for leg = {'leading', 'trailing'}
        leg = leg{:};
        fbd = fbd.addFrame('pelvis_y', [leg '_leg'], 'revolute');
        fbd = fbd.addRod([leg '_leg'], [leg '_rod'], l, 0, 'y', ...
            trvec2tform([0 -l/2 0]) * axang2tform([0 0 1 pi/2]));
        fbd = fbd.addPoint([leg '_leg'], [leg '_mass'], 0, m, 'y', ...
            trvec2tform(mass_pos));
    end
    
    % ground
    fbd = fbd.addWall('floor', [0 1], 0, 'l2');
    
    % construct initial condition
    fbd.Configuration = [x0 y0 -phi phi]';
    fbd.Velocity = [dx0 dy0 dphi0 dphi0]';
    
    % set up labelling callback
    label_callback = @(f, s) f ...
        .labelVelocity('trailing_rod', 'r', v_of_stage(s), 'r');

    % run experiment
    TwoContactExperiment(fbd, mu_mat, M_Routh, N_Routh, h, ...
        label_callback, varargin{:});
end

function RAMone(varargin)

    % dimensions
    m1       = 7.9026;            % [kg] mass of the main body
    m2       = 0.7887;            % [kg] mass of the upper leg segments
    m3       = 0.510;             % [kg] mass of the lower leg segments
    j1       = 0.08;              % [kg*m^2] inertia of the main body
    j2       = 0.00221;           % [kg*m^2] inertia of the upper leg segments
    j3       = 0.00652;           % [kg*m^2] inertia of the lower leg segments
    s1       = 0.137675;          % [m] Distance from the hip axis to pitch axis
    s2       = 0.019399;          % [m] distance between hip joints and CoG of the upper leg segments
    s3       = 0.2385 - 0.073265; % [m] distance between foot points and CoG of the lower leg segments
    l2       = 0.2;               % [m] length of the upper leg segments
    l3       = 0.2385;            % [m] length of the lower leg segments
    rFoot    = 0.0563/2;          % [m] foot radius
    
    % forward kinematics
    % joints
    p_hip_jt = [0 -s1 0];
    p_calf_jt = [0 -l2 0];
    p_foot = [0 -l3 0];
    
    % rods
    r_rod = [0 0 1 pi/2];
    
    p_pel_rod = p_hip_jt / 2;
    p_hip_rod = p_calf_jt / 2;
    p_calf_rod = p_foot / 2;
    
    % masses
    p_hip_mass = [0 -s2 0];
    p_calf_mass = [0 -s3 0];
       
    % initial condition components
    
    % torso
    theta0 = deg2rad(16);
    
    % leading knee
    b_l0 = deg2rad(-2);% -pi/2;
    
    % virtual leg stance angle
    phi0 = deg2rad(70.0);
    
    % hip angles
    a_t0 = + phi0;
    a_l0 = - phi0;
    
    % trunk height s.t. feet touch ground
    leg_ht = l3 * cos(theta0 + a_l0 + b_l0) + l2 * cos(theta0 + a_l0);
    y0 = leg_ht + s1 * cos(theta0) + rFoot;
    
    % trailing knee s.t. foot touches ground
    back_knee = leg_ht - l2 * cos(theta0 + a_t0);
    g_t0 = asin(back_knee/l3);
    b_t0 = -a_t0 - theta0 + g_t0 - pi/2;
    
    % friction
    mu = 1e5;
    mu_mat = diag([mu mu]);
    
    % sim params
    h = 1;
    N_Routh = 10;
    M_Routh = 2 ^ 20;
    
    
    % construct system
    fbd = FreeBodyDiagram();
    
    % ground
    fbd = fbd.addWall('floor', [0 1], 0, 'l2');
    
    % pelvis
    fbd = fbd ...
        .addFrame('base', 'pelvis', 'floating') ...
    	.addRod('pelvis', 'pelvis_rod', s1, 0, 'y', trvec2tform(p_pel_rod) * axang2tform(r_rod), false) ...
    	.addPoint('pelvis', 'pelvis_mass', j1, m1, 'y');
    
    % legs
    for leg = {'leading', 'trailing'}
        hip = [leg{:} '_hip'];
        calf = [leg{:} '_calf'];
        
        % hip
        fbd = fbd ...
            .addFrame('pelvis', hip, 'revolute', trvec2tform(p_hip_jt)) ...
        	.addRod(hip, [hip '_rod'], l2, 0, 'y', trvec2tform(p_hip_rod) * axang2tform(r_rod), false) ...
        	.addPoint(hip, [hip '_mass'], j2, m2, 'y', trvec2tform(p_hip_mass), false);
        
        %calf
        fbd = fbd ....
            .addFrame(hip, calf, 'revolute', trvec2tform(p_calf_jt)) ...
        	.addRod(calf, [calf '_rod'], l3, 0, 'y', trvec2tform(p_calf_rod) * axang2tform(r_rod), false) ...
        	.addPoint(calf, [calf '_mass'], j3, m3, 'y', trvec2tform(p_calf_mass), false) ...
        	.addDisc(calf, [calf '_foot'], rFoot, 0, 'y', trvec2tform(p_foot));
        
    end
    
    
    % construct initial configuration
    % configuration space is [x, y, theta, a_l, b_l, a_t, b_t]
    fbd.Configuration = [0 y0 theta0 a_l0 b_l0 a_t0 b_t0]';
    
    % construct impact terms
    [Jn, Jf] = fbd.contactJacobians();
    
    % construct initial velocity such that trailing foot is stationary
    fbd.Velocity = [-Jf(2, 3) -Jn(2, 3) 1 0 0 0 0]';
    
    % set up labelling callback
    label_callback = @(f,s) f;
    
    % run experiment
    TwoContactExperiment(fbd, mu_mat, M_Routh, N_Routh, h, ...
        label_callback, varargin{:});
end

function l = v_of_stage(stage)
    l = '$v$';
    if stage == -1
        l = '$v^-$';
    end
    if stage == 1
        l = '$v^+$';
    end
end

function BallStack(varargin)

    % dimensions
    % mass
    m = 1;
    
    % radii
    R = 1;
       
    % stack_height
    H = 2;
    
    % number of balls;
    csum = @(n) round(n * (n + 1) / 2);
    N = csum(H);
    
    % friction enough to stick
    mu = sqrt(3);
    m = 3 * csum(H - 1) + H;
    mu_mat = mu * eye(m);
    
    % initial velocity
    v0 = 1;
    
    % sim params
    h = 1;
    N_Routh = 10 / h; 
    M_Routh = 2 ^ 20;
    
    
    % construct system
    fbd = FreeBodyDiagram();
    
    % ground
    fbd = fbd.addWall('floor', [0 1], 0, 'l2');
    
    q0 = zeros(1, 3 * N);
    
    % pelvis
    ball_num = 0;
    for v=(H-1):-1:0
        for p = 0:(H-v-1)
            ball_name = ['ball_' int2str(ball_num)];
            pos = ...
                R * ([0 1] + 2 * v * [cosd(60) sind(60)] + 2 * p * [1 0]);
            fbd = fbd ...
                .addFrame('base', ball_name, 'floating') ...
                .addDisc(ball_name, [ball_name '_disc'], R, m, 'y');
            q0((1:2) + 3 * ball_num) = pos;
            ball_num = ball_num + 1;
        end
    end
            
    
    % construct initial configuration
    % configuration space is [x1, y1, theta1, ... xN, yN, thetaN]'
    fbd.Configuration = q0';
    %fbd.view();
    
    % construct initial velocity such that trailing foot is stationary
    velocity = zeros(1, 3 * N);
    velocity(2) = -v0;
    fbd.Velocity = velocity';
    
    % set up labelling callback
    label_callback = @(f, s) f;
    
    Z = zeros(3);
    XF = diag([-1 1 -1]);
    symmetry = [XF  Z  Z;
                 Z  Z XF;
                 Z XF  Z];
    
    % run experiment
    SymmetricContactExperiment(fbd, mu_mat, M_Routh, N_Routh, h, ...
        label_callback, symmetry, varargin{:});
end

function SymmetricContactExperiment(fbd, mu_mat,  M_Routh, N_Routh, h, ...
    label_cb, symmetry, skip_run)

    % extract figure name prefix from parent function
    prefix = dbstack(1).name;
    save_file = [prefix '_data.mat'];
    
    % load data if only plotting
    if nargin < 8
        skip_run = false;
    end
    
    if skip_run
        load(save_file);
    else
        tic;
        [v_s, v_a, v_r] = ...
            CompareImpactMethods(fbd, mu_mat, M_Routh, N_Routh, h, false);
        toc
        save(save_file, '-regexp', '^(?!(skip_run|label_cb)$).');
    end
    
    for i=1:length(v_s)
        v_s{i} = v_s{i}(:,1);
    end
    n_r = length(v_r);
    v_r_new = cell(1, 2 * n_r);
    v_r_new(1:n_r) = v_r;
    for i = (1:n_r)
        v_r_new{i + n_r} = symmetry * v_r_new{i};
    end
    
    % plot figures
    PlotImpacts(fbd, v_s, v_a, v_r_new, prefix, label_cb);
end

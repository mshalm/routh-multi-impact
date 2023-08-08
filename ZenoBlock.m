function ZenoBlock()
    % get freebody diagram
    [~, fbd] = Phone(false, true);
    
    omega_0 = 1;
    N_CYCLES = 10;
    [t, z, tf1, tf2, Lnf] = NZenoCycle(omega_0, N_CYCLES);
    x = z(:,1:6);
    s = t + z(:, 7) + z(:, 9);
  
    % initial condition
    x0 = x(1, :)';
    
    % calculate apex state x^*
    % apex happens when all of kinetic energy has been transformed into
    % potential energy, i.e. m*g*l*(sin(theta^*+alpha) - sin(alpha)) = KE_0
    [a, b, m, ~, ~] = PhoneParameters();
    [l, I] = l_I_from_a_b_m(a, b, m);
    KE_0 = 0.5*(I + m*l^2)*omega_0^2;
    alpha = angle_alpha(a, b);
    theta_star = asin(KE_0/(m*gravity()*l) + sin(alpha)) - alpha
    x_star = rolling_state(theta_star, 0, a, b);
    
    % pre-impact state
    x_pre = x0 .* [1 1 1 -1 -1 -1]';
    
    % retrieve post-impact state
    first_cycle = t <= tf1(1);
    x_first_cycle = x(first_cycle, :);
    x_post = x_first_cycle(end, :)';
    
    % by balance of angular momentum, we find the reduction in omega by
    % impact:
    % (I + ml^2*cos(2*pi-2*alpha)dtheta_pre = (I + ml^2)dtheta_post = 0.7
    reduction = (I + m*l^2*cos(pi - 2*alpha))/(I + m*l^2);
    GT_reduction = 0.7;
    assert(reduction == GT_reduction);
    
    states = {x0, x_star, x_pre, x_post};
    points = {1, 1, 1, 2};
    spins = {1, [], 1, 2};
    labels = {'$\dot \theta_0$', '', '$-\dot \theta_0$', ...
        ['$-', char(string(GT_reduction)),'\dot \theta_0$']};
    prefix = 'Zeno_';
    names = {'0', 'apex', 'pre', 'post'};
    
    
    % Run once to get consistent axes limits for free body diagram plots.
    % Then, run again to save them.
    lim_set = {};
    for do_save = [false true]
        
        limits = zeros(0, 4);
        for i=1:4
            fbd = setup_state(fbd, states{i});
            fbd = fbd.view(@(f, stage) DiagramLabelCallback(...
                f, points{i}, spins{i}, labels{i}), lim_set{:});
            limits = [limits; xlim(gca) ylim(gca)];
            if do_save
                fbd.Styler.print([prefix names{i}]);
            end
            fbd.Styler.close();
        end
        
        limits = [min(limits(:,1)), max(limits(:,2)), ...
                  min(limits(:,3)), max(limits(:,4))];
        lim_set = {limits};
        
    end
    
    
    % extend past Zeno
    % find accumulation point in t, s by geometric continuation
    dtf2 = tf2(2:end) - tf2(1:end-1);
    sf = Lnf(:, 1) + Lnf(:, 2) + tf2;
    dsf = sf(2:end) - sf(1:end-1);
    frac_add_s = dsf(2:end) ./ dsf(1:end-1);
    assert(abs(frac_add_s(end) - GT_reduction^2) < 1e-5);
    s_end_approx = sf(end-1) + (1/(1 - GT_reduction^2))*dsf(end)
    t_end_approx = tf2(end-1) + (1/(1 - GT_reduction^2))*dtf2(end)
    
    DT_eq = 0.5;
    t_eq = (0:0.01:DT_eq)';
    x_zeno = ones(size(t_eq)) * [x0(1:3); zeros(3,1)]';
    t_new = t_eq + t_end_approx;
    lambda = m*gravity()*[0.5 0 0.5 0];
    Lambda = t_eq * lambda + z(end, 7:10);
    z_new = [x_zeno Lambda];
    
    t = [t; t_new];
    z = [z; z_new];
    s = t + z(:, 7) + z(:, 9);
    
    % get relative accumulation rates \dot t, lambda_n
    ds = s(2:end) - s(1:end-1);
    dt = t(2:end) - t(1:end-1);
    dLn1 = z(2:end, 7) - z(1:end-1, 7);
    dLn2 = z(2:end, 9) - z(1:end-1, 9);
    dt_ds = dt ./ ds;
    dLn1_ds = dLn1 ./ ds;
    dLn2_ds = dLn2 ./ ds;
    
    styler = PlotStyler();
    hold on;

    plot(s(1:end-1), dt_ds, styler.linestyles{1});
    plot(s(1:end-1), dLn1_ds, styler.linestyles{2});
    plot(s(1:end-1), dLn2_ds, styler.linestyles{3});

    legend('$\dot t$', '$\lambda_{n,A}$', '$\lambda_{n,B}$');

    title('Time \& Impulse Accumulation Rates');
    xlabel('$s$ [sec]');
    styler.asSeries();
    styler.print([prefix 't_Lambda_rates']);
    
    
    % plot time series:
    % theta vs. s
    % theta vs. t
    % \dot theta vs. s
    % \dot theta vs. t
    % t, Lambda_n vs. s
    all_t = t > -1;
    domains = {all_t, all_t, all_t};
    z_t = [z t];
    idxs = {[3 6], [3 6], [11 7 9]};
    state_leg = {'$\theta$','$\dot\theta$'};
    legends = {state_leg,state_leg,{'t', '$\Lambda_{n,1}$', '$\Lambda_{n,2}$'}};
    titles = {'$s$-domain Trajectrory', '$t(s)$-domain Trajectory', ...
        'Time \& Impulse Evolution'};
    
    is_s = {true false true};
    names = {'state_s', 'state_t', 't_Lambda_s'};
    
    for i=1:3
        styler = PlotStyler();
        hold on;
        
        % set x label and axis
        if is_s{i}
            x_var = s;
            xlabel('$s$ [sec]');
        else
            x_var = t;
            xlabel('$t(s)$ [sec]');
        end
        
        qq = 1;
        for j = idxs{i}
            plot(x_var(domains{i}), z_t(domains{i}, j), ...
                styler.linestyles{qq});
            qq = qq + 1;
        end
        
        if numel(legends{i}) > 0
            legend(legends{i}{:});
        end
        
        if ~strcmp(titles{i},'')
            title(titles{i});
        end
        styler.asSeries();
        styler.print([prefix names{i}]);
    end
end

function fbd = DiagramLabelCallback(fbd, pos_inds, ang_vel_ind, ang_vel_label)
    [~, ~, points, bodies] = fbd.contactTerms();
    alphabet = {'A ', 'B '};
    if ang_vel_ind == 1
        omegaalign = 'tr';
        pointalign = 'bl';
    else
        omegaalign = 'tl';
        pointalign = 'bl';
    end
    
    for i = pos_inds
        fbd = fbd.labelPoint(bodies{i}, points{i}, alphabet{i}, ...
            'r', 0, pointalign);
    end
    
    fbd = fbd.labelVelocity('phone', [0 0]', '$v$', 'r');
    
    
    for i = ang_vel_ind
        fbd = fbd.labelAngularVelocity(bodies{i}, points{i}, 0.2, ...
            ang_vel_label, 'r', omegaalign);
    end
end

function [t, z, tf1, tf2, Lnf] = NZenoCycle(omega_0, N)
    % Simulate N double-impact rocking cycles.
    tf = 0;
    Lamdbaf = zeros(1, 4);
    t = zeros(0, 1);
    z = zeros(0, 10);
    tf1 = zeros(N, 1);
    tf2 = zeros(N, 1);
    Lnf = zeros(0, 2);
    omega = omega_0;
    for i=1:N
        [ti0, zi0, tf1i, tf2i] = SingleZenoCycle(omega);
        ti = ti0 + tf;
        zi = [zi0(:,1:6) (zi0(:, 7:10) + Lamdbaf)];
        t = [t; ti(2:end)];
        z = [z;
             zi(2:end, :)];
        tf1(i) = tf1i + tf;
        tf2(i) = tf2i + tf;
        tf = t(end);
        Lamdbaf = z(end, 7:10);
        Lnf = [Lnf; z(end, [7 9])];
        omega = zi(end, 6);
    end

end

function [t, z, tf1, tf2] = SingleZenoCycle(omega_0)
    % Simulate a rocking cycle of left and right impacts
    
    % Simulate left half
    [t1, z1, tf1] = SingleZenoLeftHalfCycle(omega_0);
    omega_1 = -z1(end, 6);
    
    % By symmetry, simulate right half as a reflection of the left half.
    [t20, z2f, ~] = SingleZenoLeftHalfCycle(omega_1);
    
    % flip horizontally, inverse rotation;
    xflip = [-1 1 -1];
    q2 = xflip .* z2f(:, 1:3);
    v2 = xflip .* z2f(:, 4:6);
    
    % flip contacts and horizontal impulses
    lflip = [1 -1];
    L2_2 = lflip .* z2f(:, 7:8) + z1(end, 9:10);
    L1_2 = lflip .* z2f(:, 9:10) + z1(end, 7:8);
    z2 = [q2 v2 L1_2 L2_2];
    t2 = t20 + tf1;
    
    % concatenate and remove redundant state
    t = [t1; t2(2:end)];
    z = [z1; z2(2:end, :)];
    tf2 = t(end);

end

function [l, I] = l_I_from_a_b_m(a, b, m)
    % radius of pendulum
    l = sqrt(a^2 + b^2)/ 2;
    
    % moment of inertia
    I = m * (a^2 + b^2) / 12;
end

function [t, z, tf] = SingleZenoLeftHalfCycle(omega_0)
    % Simulate half of Zeno cycle with sustained contact on left corner.

    % get phone results
    [~, fbd] = Phone(true, true);
    
    % phone system parameters
    % width, height, mass, friction, drop velocity
    [a, b, m, mu, ~] = PhoneParameters();
    
    [l, I] = l_I_from_a_b_m(a, b, m);
    
    % permanently turn on bottom 2 contacts
    fbd.PhiTolerance = b/2;
    M = fbd.massMatrix();
    
    
    % actual phi tolerance used in contact law
    assert(mu == 1);
    mu = mu * ones(2,1);
    
    % simulation pendulum portion of dynamics
    [t_pend, theta, dtheta] = pendulum(omega_0, a, b, m, l, I);

    % solve for corresponding state and contact force for pendulum motion.
    lambda = zeros(numel(t_pend), 2);
    x_pend = zeros(numel(t_pend), 6);
    for i=1:numel(t_pend)
        x_pend(i, :) = rolling_state(theta(i), dtheta(i), a, b)';
        lambda(i, :) = point_A_stopping_force(x_pend(i, :)', fbd, mu);
    end
    
    % integrate force to recover impulse
    Lambda_1_pend = cumtrapz(t_pend, lambda);
    Lambda_2_pend = zeros(size(Lambda_1_pend));
    
    % solve impact
    xf = x_pend(end, :)';
    Lamdba_1f = Lambda_1_pend(end, :);

    tf = t_pend(end);
    % find impact impulse Lambda_2 that halts contact 2, i.e.
    % 0 = J_2 v^+ = J_2 (v^- + inv(M) J_2');
    fbd = setup_state(fbd, xf);
    [Jn, Jf, ~, ~, ~, ~] = fbd.contactTerms();
    Jn = Jn(2, :);
    Jf = Jf(2, :);
    J = [Jn; Jf];
    D = (J * (M \ J'));
    D = 0.5 * (D + D');
    b = -J * fbd.Velocity;
    Lambda_2 = D \ b;
    
    % verify Lambda_2 is in the friction cone
    assert(abs(Lambda_2(2))<= mu(2) * Lambda_2(1));
    
    % interpolate impact states with time, config frozen to preimpact.
    N = 11;
    t_impact = tf * ones(N, 1);
    Lambda_2_impact = linspace(1/N, 1, N)' * Lambda_2';
    v_impact = xf(4:6)' + (M \ (J' * Lambda_2_impact'))';
    q_impact = xf(1:3)' .* ones(size(v_impact));
    x_impact = [q_impact v_impact];
    
    % contact 1 does not accumulate impulse during impact
    Lambda_1_impact = Lamdba_1f .* ones(size(v_impact(:,1:2)));
    t = [t_pend; t_impact];
    z = [x_pend Lambda_1_pend Lambda_2_pend;
         x_impact Lambda_1_impact Lambda_2_impact];
    
end

function g = gravity()
    g = 9.81;
end

function alpha = angle_alpha(a, b)
    alpha = atan(b/a);
end

function [t, theta, dtheta, ddtheta] = pendulum(omega_0, a, b, m, l, I)
    % Simulate pendulum dynamics of body:
    % KE = 0.5(I + ml^2)\dot theta^2,
    % PE = mgl(sin(alpha + theta) - sin(alpha)),
    % alpha = atan(b/a),
    % l is length of pendulum, sqrt(a^2 + b^2)/2
    % From lagraniang dynamics, d P / d theta + d K / d (\dot theta) = 0
    % mglcos(alpha + theta) + (I + ml^2) \ddot theta = 0
    g = gravity();
    alpha = angle_alpha(a, b);
    ydot_fun = @(z) [z(2, :); -(m*g*l*cos(z(1, :) + alpha))/(I + m*l^2)];
    ode_fun = @(t, z) ydot_fun(z);
    % terminate when theta returns to zero.
    options = odeset('Events', @theta_event, ...
        'RelTol', 1e-8, 'AbsTol', 1e-8);
    y0 = [0 omega_0]';
    [t, y, ~, ~, ie] = ode45(ode_fun, [0, 2*omega_0], y0, options);
    
    % double check theta came back to zero.
    assert(ie == 1);
    theta = y(:, 1);
    dtheta = y(:, 2);
    ydot = ydot_fun(y')';
    ddtheta = ydot(:, 2);
end

function fbd = setup_state(fbd, x)
    % set state in freeBodyDiagram.
    q = x(1:3);
    v = x(4:6);
    fbd.Configuration = q;
    fbd.Velocity = v;
end

function x = rolling_state(theta, omega, a, b)
    % Reconstruct state of pivoting about 
    pt_ground = -[a; 0]/2;
    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];
    r = R * [a; b]/2;
    pdot3d = cross([0;0; omega], [r; 0]);
    q = [pt_ground + r; theta];
    v = [pdot3d(1:2); omega];
    x = [q; v];
end

function [value, terminal, direction] = theta_event(~, y)
    value = y(1);
    terminal = 1;
    direction = -1;
end

function lambda = point_A_stopping_force(x, fbd, mu)
    fbd = setup_state(fbd, x);
    M = fbd.massMatrix();
    [Jn, Jf, ~, ~, ~, bias] = fbd.contactTerms();
    Jn = Jn(1, :);
    Jf = Jf(1, :);
    J = [Jn; Jf];
    bias = bias(1, :)';
    accel_noncontact = fbd.forwardDynamics();
    
    D = (J * (M \ J'));
    D = 0.5 * (D + D');
    b = -(J * accel_noncontact + bias);
    lambda = D \ b;
    assert(abs(lambda(2))<= mu(1) * lambda(1));
end



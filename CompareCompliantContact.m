function CompareCompliantContact(skip_run)
    % minimum stiffness of 2 contacts
    stiffness = 1e6;
    
    % number of samples
    N = 51;
    
    % stiffness sensitivity: nominial initial state, compliant contact with
    % min(k_A,k_B) == 1e6, log(k_A/k_B) ranging from -3 to 3.
    
    % run phone routh example
    Phone(skip_run, true);
    stiffness_limit = 5;
    range = linspace(-stiffness_limit, stiffness_limit, N);
    type = 0;

    CompliantImpactRun(stiffness, range, type);
    
    % initial condition sensitivity, with initial angle varying bewtween
    % [-theta_limit, theta_limit], initial downward velocity, and constant
    % initial mechanical energy (KE + PE)
    theta_limit = 1e-2 * (pi/180);
    range = linspace(-theta_limit, theta_limit, N);
    type = 1;
    CompliantImpactRun(stiffness, range, type);
    
end

function CompliantImpactRun(stiffness, range, type)
    % get phone results
    [v_Routh, fbd, v_Sequential] = Phone(true, true);
    
    g = 9.81;
    
    % phone system parameters
    % width, height, mass, friction, drop velocity
    [a, b, m, mu, vn0] = PhoneParameters();
    
    %Jn = fbd.contactTerms();
    x0 = [fbd.Configuration; fbd.Velocity];
    terms_callback = @(x) contact_terms(x, a, b, m, g);
    [~, ~, Jn, ~, ~, ~] = terms_callback(x0);
    
    % permanently turn on bottom 2 contacts
    fbd.PhiTolerance = b/4;
    % actual phi tolerance used in contact law
    
    mu = mu * ones(2,1);
    velocity_term_eps = 1e-3 * m/stiffness;
    impulse_term_eps = 1e-2 * m * abs(vn0);
    
    % ODE state: [q; v; \int lambda_n]; 
    q0_nominal = fbd.Configuration;
    v0_nominal = fbd.Velocity;
    y0_nominal = [q0_nominal; v0_nominal; 0; 0];
    
    nruns = numel(range);

    if type == 0
        % vary k1/k2, while holding min(k1,k2) == stiffness
        order_shifts = 0.5*([range; -range] - min(range,-range));
        stiffnesses = stiffness * 10.^order_shifts;
        theta0 = zeros(1, nruns);
        plotrange = 10.^range;
        is_log = true;
        label = 'Stiffness ratio $\frac{k_A}{k_B}$';
        plot_title = 'Varying Stiffness Ratio';
        fig_name = 'ratio_sweep';
    else
        stiffnesses = stiffness * ones(2, nruns);
        theta0 = range;
        plotrange = range * 180/pi;
        is_log = false;
        label = 'Initial block angle $\theta_0$ [deg]';
        plot_title = 'Varying Initial Condition';
        fig_name = 'angle_sweep';
    end
    
    nx = numel(fbd.Velocity) + numel(fbd.Configuration);
    
    x_final = zeros(nx, nruns);
    x_all = cell(nruns, 1);
    Lambda_all = cell(nruns , 1);
    t_all = cell(nruns, 1);
    
    normal_law = @(phi, v_n, k) OverdampedKelvinVoigt(phi, v_n, k, m);
    term_event = @(t,y) normal_separation(t, y, velocity_term_eps, ...
        impulse_term_eps);
    
    tspan = [0, 1e-1];
    
    tic;
    for j = 1:nruns
        k = stiffnesses(:, j);
        y0 = initial_state(terms_callback, y0_nominal, theta0(j), g);
        ode_fun = ...
            @(t,y) compliant_ode(t, y, k, mu, normal_law, terms_callback);
        [t, y] = sim_till_term(ode_fun, tspan, y0, term_event);
        xf = y(end, 1:6);
        t_all{j} = t;
        x_all{j} = y(:, 1:6);
        Lambda_all{j} = y(:, 7:end);
        x_final(:, j) = xf;
    end
    all_runs_duration = toc;
    
    Lambda_1first = Lambda_all{end};
    t_2started = find(Lambda_1first(:,2) > 0);
    t_2start = t_2started(1);
    Lambda_1_remaining_at_Lambda_2_start =...
        1 - Lambda_1first(t_2start, 1) / Lambda_1first(end, 1);
    percent_Lambda_1_first = 100 - Lambda_1_remaining_at_Lambda_2_start*100

    fprintf("Compliant Impact Performance:\n total time %f\n average time %f\n\n", ...
        all_runs_duration, all_runs_duration/nruns);
    
    
    hold off;
    % characterize most asymmetric impact
    for j = []%nruns
        x = x_all{j};
        t = t_all{j};
        Lambda = Lambda_all{j};
        lambda = abs(diff(Lambda) ./ diff(t));
        nt = numel(t); 
        vn = zeros(2, nt);
        vt = zeros(2, nt);
        for n = 1:nt
            [vn(:, n), vt(:, n)]  = get_v_n_v_t(x(n, :)');
        end
        styler = PlotStyler();
        plot(t, vn');
        title('$v_n$');
        styler.asSeries();
        styler = PlotStyler();
        plot(t, vt');
        title('$v_t$');
        styler.asSeries();
        styler = PlotStyler();
        plot(t, Lambda);
        title('$\Lambda_n$');
        styler.asSeries();
        styler = PlotStyler();
        semilogy(t(1:end-1), lambda);
        title('$\lambda_n$');
        styler.asSeries();
        styler = PlotStyler();
        plot(t, x(:, 1:2));
        title('$p$');
        styler.asSeries();
        styler.asSeries();
        styler = PlotStyler();
        plot(t, x(:, 3));
        title('$\theta$');
        styler.asSeries();
        styler.asSeries();
        styler = PlotStyler();
        plot(t, x(:, 6));
        title('$\dot\theta$');
        styler.asSeries();
    end
    
    
    
    
    styler = PlotStyler();
    styler.linewidth = 6;
    styler.linestyles = PlotStyler.SOLID;
    hold on;
    % underlay Routh
    v_Routh_ends = zeros(2, length(v_Routh));
    for i = 1:length(v_Routh)
        v_Routh{i} = Jn * v_Routh{i};
        v_Routh_ends(:, i) = v_Routh{i}(:, end);
    end
    keeps = DataTools.SparseScatter(v_Routh_ends, 100);
    v_Routh_ends = v_Routh_ends(:, keeps);
    
    styler.plotPoint(v_Routh_ends, '', 'm2', ...
        0, PlotStyler.LABELALIGNMENT, 1);
   

    v_n_f = zeros(nruns, 2);
    for j = 1:nruns
        v_n_f(j, :) = get_v_n_v_t(x_final(:, j));
    end
    
    compliant_marker_scale = 0.5;
    plot_marker_scale = 50;
    styler.plotPoint(v_n_f', '', plotrange, 0, ...
        PlotStyler.LABELALIGNMENT, compliant_marker_scale);
    Plot_Routh = plot(nan, nan, '-o', ...
            'MarkerSize', plot_marker_scale, ...
            'Color', [1 1 1], ...
            'MarkerFaceColor', PlotStyler.colorComponents('m2'));
    title(plot_title);
    xlabel('A Normal Velocity');
    ylabel('B Normal Velocity');
    styler.colorbar(label, is_log);
    if is_log
        caxis([min(plotrange) max(plotrange)]);
    end
    styler.asCurve();
    legend([Plot_Routh],{'Ours'}, 'Location', 'northeast');
    styler.print(['Compliant_' fig_name]);
end

function [t,y] = sim_till_term(ode_fun, tspan, y0, term_event)
    eventsfun = term_event;
    options = odeset('Events', eventsfun,'RelTol', 1e-10, 'AbsTol', 1e-11);
    [t, y, ~, ~, ie] = ode15s(ode_fun, tspan, y0, options);
    assert(ie(end) == 1);
end

function fbd = setup_state(fbd, x)
    q = x(1:3);
    v = x(4:6);
    fbd.Configuration = q;
    fbd.Velocity = v;
end

function [y0] = initial_state(terms_callback, y0_nominal, theta0, g)
    % step 1: change theta0 in nominal initial state
    x0 = y0_nominal(1:6);
    v0_nominal = x0(4:6);
    x0(3) = theta0;
    
    % step 2: get penetration depth
    [~, ~, ~, ~, phi, ~] = terms_callback(x0);
    pen_depth = abs(min(min(phi), 0));
    
    % step 3: raise initial state
    x0(2) = x0(2) + pen_depth;
    
    % step 4: reduce initial kinetic energy to hold initial total energy
    % constant
    KE_over_m_nominal = 0.5 * v0_nominal(2)^2;
    PE_increase_over_m = pen_depth*g;
    KE_ove_m_new = KE_over_m_nominal - PE_increase_over_m;
    dot_y_0_new = sqrt(2 * KE_ove_m_new)*sign(v0_nominal(2));
    x0(5) = dot_y_0_new;
    
    % TODO: decide whether or not to reduce velocity to keep total energy
    % constant
    y0 = [x0; y0_nominal(7:8)];
end

function [v_n, v_t] = get_v_n_v_t(x)
    [~, ~, Jn, Jf, ~, ~] = contact_terms(x, 1, 2, 1, 9.81);
    v_n = Jn * x(4:6);
    v_t = Jf * x(4:6);

end

function [value, terminal, direction] = normal_separation(...
    t, y, vtol, itol)
    % termination condition: all active contacts are separating, and some
    % minimum impulse has been applied already at each corner
    % TODO: add min impluse condition
    x = y(1:6);
    v = x(4:6);
    impulse_n = y(7:8);
    [~, ~, Jn, ~, phi, ~] = contact_terms(x, 1, 2, 1, 9.81);
    v_n = Jn * v;
    active = phi <= 0;
    v_n = [v_n(active); 1];
    velocity_value = min(v_n) + vtol;
    impulse_value = min(impulse_n) - itol;
    value = min(velocity_value, impulse_value);
    terminal = 1;
    direction = 1;
end

function [M, accel_noncontact, Jn, Jf, phi, bias] = contact_terms(x, a, b, m, g)
    I = (1/12)*(a^2 + b^2);
    q = x(1:3);
    v = x(4:6);
    p_WoBo_W = q(1:2);
    theta = q(3);
    omega = v(3);
    R_WB = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    cross_z = [0 -1; 1 0];
    p_BoBi_B = [-a   a
                -b  -b]/2;
    p_BoBi_W = R_WB * p_BoBi_B;
    p_WoBi_W = p_WoBo_W + p_BoBi_W;
    phi = p_WoBi_W(2, :)';
    
    
    M = diag([m m I]);
    accel_noncontact = [0; -g; 0];
    J_1 = [eye(2) cross_z * p_BoBi_W(:, 1)];
    J_2 = [eye(2) cross_z * p_BoBi_W(:, 2)];
    Jn = [J_1(2, :);
          J_2(2, :)];
    Jf = [J_1(1, :);
           J_2(1, :)];
    bias_W = -p_BoBi_W' * omega^2;
    bias = [bias_W(:, 2), bias_W(:, 1)];
    
end

function y_dot = compliant_ode(t, y, k, mu, normal_law, terms_callback)
    x = y(1:6);
    v = x(4:6);
    [M, accel_noncontact, Jn, Jf, phi, bias] = terms_callback(x);
    active = phi <= 0;
    Jn = Jn(active, :);
    Jf = Jf(active, :);
    phi = phi(active, :);
    bias = bias(active, :);
    
    v_n = Jn * v;
    lambda_n = normal_law(phi, v_n, k(active));
    v_t = Jf * v;
    accel_normal = accel_noncontact + M \ (Jn' * lambda_n);

    lambda_f = -mu(active, :) .* abs(lambda_n) .* robustSign(v_t);
    accel = accel_normal + M \ (Jf' * lambda_f);
    
    x_dot = [v; accel];
    lambda_n_all = zeros(2,1);
    lambda_n_all(active) = lambda_n;
    y_dot = [x_dot; lambda_n_all];
end

function vhat = robustSign(v)
    vhat = v./max(abs(v), 1e-10);
end

function lambda_n = OverdampedKelvinVoigt(phi, v_n, k, m)
    zeta = 5;
    damping = 2 * zeta * sqrt(k./m);
    lambda_n = - (phi <=0) .* min((phi .* k + v_n .* damping), 0);
end

    


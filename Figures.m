function Figures
    ExampleFigs = ...
        {@UnitGraph;
         @UnitPhasePortrait;
         @OneContactRouth;
         @BallFall;
         @BallPhiPhase};
    for i=1:length(ExampleFigs)
        styler = PlotStyler();
        styler.fontsize = 18;
        styler = ExampleFigs{i}(styler);
        styler.print(func2str(ExampleFigs{i})).close();
    end
end

function styler = UnitGraph(styler)
    styler.linewidth = 6;
    hold on;
    points = [-1  0  0  1;
              -1 -1  1  1];
    
    plot(points(1, :), points(2, :));
    axis tight;
    ylim(PlotStyler.padAxis(ylim(gca), PlotStyler.REL_PADDING));
    title('Unit$(v)$ Graph');
    xlabel('Unit$(v)$');
    ylabel('$v$');
    
    styler.asCurve(0, 0);
end

function styler = UnitPhasePortrait(styler)
    hold on;
    lims = [-1 1 -1 1];
    
    % vector field
    dx = 2 ^ (-2);
    [X, Y] = meshgrid(lims(1):dx:lims(2), lims(3):dx:lims(4));
    points = [X(:) Y(:)]';
    norms = vecnorm(points);
    norms(norms == 0) = inf;
    arrows = - points ./ norms;
    styler.plotArrow(points, arrows, '', 'y');
    
    % trajectories
    starts = [-0.4  0.7;
               0.7  0.2];
    ends = starts .* [0.3 0];
    X = [starts(1, :); ends(1, :)];
    Y = [starts(2, :); ends(2, :)];
    
    %reset color order
    set(gca, 'ColorOrderIndex', 1);
    lines = plot(X, Y);
    set(gca, 'ColorOrderIndex', 1);
    pt1 = styler.plotPoint(ends(:,1));
    pt2 = styler.plotPoint(ends(:,2));
    
    axis tight;
    title('Trajectories of $\dot v \in -$Unit$(v)$');
    xlabel('$v_1$');
    ylabel('$v_2$');
    
    styler.asCurve(0, 0);
end

function styler = OneContactRouth(styler)
    styler.linewidth = 6;
    styler.fontsize = 12;
    hold on;
    
    % generate trajectory
    R = @(rot) [cosd(rot) -sind(rot); sind(rot)  cosd(rot)];
    
    % cone
    J1 = R(-25) * [0 1]';
    J2 = R(105) * J1;
    
    % trejectory
    P1 = [0 0]';
    P2 = [0 -1]';
    P3 = P2 - 2*J2;
    P = [P1 P2 P3];
    PX = P(1, :);
    PY = P(2, :);
    
    % plot cones
    styler.plotArrow([P2 P2 P3 P3], [J1 J2 J1 J2], '', 'r', 'h');
    
    % plot trajectory
    y = PlotStyler.colorComponents('y');
    b = PlotStyler.colorComponents('b');
    plot(PX, PY, '--', 'Color', y);
    
    % label points
    styler.plotPoint(P3,'$v$', b, 0, 'b');
    styler.plotPoint(P1,'$v^+$', b, 0, 't');
    axis tight;
    
    % label axes
    styler.asDoodle([false true true true]);
    axis tight;
    xlims = xlim(gca);
    ylims = ylim(gca);
    aspect = 1.7;
    styler.aspect = aspect;
    styler.height = styler.height / aspect;
    styler.labelPoint([0 ylims(1)]', ...
        '$-J_n v$', PlotStyler.AXISCOLOR, 0, 'b');
    styler.labelPoint([xlims(2) 0]', ...
        '$J_t v$', PlotStyler.AXISCOLOR, 0, 't');
end

function styler = BallFall(styler)
    % figure parameters
    z0        =  2;
    dz0       = -3/2;
    R         =  1;
    m         =  1;
    ground_ht = -R;

    % construct 
    fbd = FreeBodyDiagram() ...
        .addWall('floor', [0 1], ground_ht, 'l2') ...
        .addFrame('base', 'object', 'floating') ...
        .addDisc('object', 'disc', R, m, 'y');
    
    fbd.Configuration = [0 z0 0]';
    fbd.Velocity = [0 dz0 0]';
    
    label_callback = @(f) f;% ...
        %.labelVelocity('disc', [0 0]', '$\dot z$', 'r');
    
    limits = [0 0 ground_ht z0] + R * [-2 2 -1 1];
    fbd = fbd.view(label_callback, limits);
    fbd.Styler.plotSpan([0 ground_ht]', [0 (z0 - R)]', '$z$', 'r');
    
    styler.close();
    styler = fbd.Styler;
end

function styler = BallPhiPhase(styler)
    hold on;
    styler.linestyles = PlotStyler.SOLID;
    styler.arrowscale = 0.6;
    % plot settings
    g = -1;
    scale = 2 * [1 abs(g)]';
    delta = 2 ^ (-2);
    values = -1:delta:1;
    b = PlotStyler.colorComponents('b');
    r = PlotStyler.colorComponents('r');
    p = PlotStyler.colorComponents('p');
    l2 = PlotStyler.colorComponents('l2');
    d2 = PlotStyler.colorComponents('d2');
    
    % regions
    e = delta / 4;
    X_free = [e  e  1  1 -1 -1;
              e -1 -1  1  1  e] .* scale;
    X_impact = [-e -1 -1 -e;
                -e -e -1 -1] .* scale;
    X_conv = [e -1 -1 -e -e  e;
              e  e -e -e -1 -1] .* scale;
    X_pen = [ 0 -1 -1  0;
              1  1 -1 -1] .* scale;
          
    regions = {X_free, X_impact, X_conv};
            
    
    region_colors = {b, r, p};
    
    region_names = {'$\bar{\mathcal X}_S$', ...
                    '$\mathrm{int}(\bar{\mathcal X}_I)$', ...
                    'otherwise'};
    
    
    xdot_free = @(x) [x(2, :); g * ones(size(x(2, :)))];
    xdot_impact = @(x) repmat([0; 1], size(x(2, :)));
    xdot_conv = @(x) (x(2, :) < 0) .* xdot_impact(x);
    region_xdot = {xdot_free, xdot_impact, xdot_conv};
    
    is_free = @(x) (x(1, :) >  0 | x(2, :) >  0);
    is_impact = @(x) (x(1, :) <= 0 & x(2, :) <  0);
    is_conv = @(x) ~(is_free(x) | is_impact(x));
    is_region = {is_free, is_impact, is_conv};
    
    [Z, DZ] = meshgrid(scale(1) * values, scale(2) * values);
    points = [Z(:) DZ(:)]';
    
    region_plots = cell(1, 3);
    region_arrows = cell(1, 3);
    for i = 1:3
        region_plots{i} = fill(regions{i}(1, :), regions{i}(2, :), ...
            region_colors{i});
        region_plots{i}.FaceAlpha = 0.3;
        region_plots{i}.EdgeAlpha = 0;
        points_i = points(:, is_region{i}(points));
        region_arrows{i} = styler.plotArrow(points_i, ...
            region_xdot{i}(points_i), '', region_colors{i});
    end
    
    % shade penetration area
    pen_shade = fill(X_pen(1, :), X_pen(2, :), l2);
    pen_shade.FaceAlpha = 0.7;
    pen_shade.EdgeAlpha = 0;
    
    % trajectory
    start = scale / 3;
    t = 0:0.01:100;
    traj = [start(1) + start(2) * t + 0.5 * g * t .^ 2;
            start(2) + g * t]';
    traj = traj(traj(:, 1) >= 0, :);
    traj = [traj; 0 0];
    
    % plot trajectorty
    set(gca, 'ColorOrderIndex', 1);
    plot(traj(:, 1), traj(:, 2), 'Color', [1 1 1]);
    
    % plot endpoint
    set(gca, 'ColorOrderIndex', 1);
    styler.plotPoint(traj(end, :)','', [1 1 1]);
    
    % style plot
    styler.asCurve(0, 0);
    xlim(scale(1) * [-1 1]);
    ylim(scale(2) * [-1 1]);
    
    % text
    legend([region_plots{:}], region_names{:}, 'location', 'southeast');
    %title('Trajectories of $\dot{\bar x} \in D(\bar x)$');
    xlabel('Signed distance $\phi(z) = z$');
    ylabel('$\dot z$');
    
    % penetration labeling
    free_pos = [0.5; 1] .* scale;
    pen_pos = [-1; 1] .* free_pos;
    txtmult = 1.2;
    styler.labelPoint(free_pos, 'Above Ground', 'k', 0, 't', txtmult);
    styler.labelPoint(pen_pos, 'Below Ground', 'k', 0, 't', txtmult);
end
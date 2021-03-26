function styler = PlotImpactTrajectory(J, v_staged, v_staged_labels, ...
    v_staged_cols, v_anitescu, v_anitescu_col, v_Routh, Jax, axesLabels)
    styler = PlotStyler();
    styler.linewidth = 6;
    styler.linestyles = PlotStyler.SOLID;
    
    hold on;
    v_routh_col = 'l2';
    v_routh_end_col = 'm2';
    
    legend_marker_size = 10;
    
    % preprocess data
    J_plot = J(Jax, :);
    axesLabels = axesLabels(Jax);
    
    for i = 1:length(v_staged)
        v_staged{i} = J_plot * v_staged{i};
    end
    
    for i = 1:length(v_anitescu)
        v_anitescu{i} = J_plot * v_anitescu{i};
    end
    
    
    % get routh trajectory lengths
    Routh_lens = zeros(1, length(v_Routh));
    v_Routh_ends = zeros(2, length(v_Routh));
    for i = 1:length(v_Routh)
        v_Routh{i} = J_plot * v_Routh{i};
        v_Routh_ends(:, i) = v_Routh{i}(:, end);
        Routh_lens(i) = size(v_Routh{i}, 2);
    end

    NX = 100;
    
    
    % sub-select trajectories with unique endpoints
    keeps = DataTools.SparseScatter(v_Routh_ends, NX);
    v_Routh_ends = v_Routh_ends(:, keeps);
    % Routh_lens = Routh_lens(keeps);
    % v_Routh = v_Routh(keeps);
    
    Routh_lens = 2 * Routh_lens - 2;
    v_Routh_traj = nan(2, sum(Routh_lens));
    clen = [0 cumsum(Routh_lens)];
    for i = 1:length(v_Routh)  
        v_Routh_traj(:, (clen(i) + 1):clen(i + 1)) = ...
            [v_Routh{i} fliplr(v_Routh{i}(:, 2:end-1))];
    end
    v_Routh_traj = DataTools.LineInpaint(v_Routh_traj, NX);

    % plot Routh trajectories
    styler.plotPoint(v_Routh_traj, '', v_routh_col);
    styler.plotPoint(v_Routh_ends, '', v_routh_end_col);

    % plot staged trajectories
    for i = 1:length(v_staged)
        v_i = v_staged{i};
        Plot_i = plot(v_i(1, :), v_i(2, :), 'Color', ...
            PlotStyler.colorComponents(v_staged_cols{i}));
        styler.plotPoint(v_i(:,end), '', v_staged_cols{i});
    end
    
    for i = 1:length(v_anitescu)
        plot(v_anitescu{i}(1, :), v_anitescu{i}(2, :), ...
            'Color', PlotStyler.colorComponents(v_anitescu_col));
    end
    
    for i = 1:length(v_anitescu)
        styler.plotPoint(v_anitescu{i}(:,end), '', v_anitescu_col);
    end
    
    % get label stylings
    Plot_staged = cell(1, length(v_staged));
    for i = 1:length(Plot_staged)
        Plot_staged{i} = plot(nan, nan, '-o', ...
            'MarkerSize', legend_marker_size, ...
            'Color', PlotStyler.colorComponents(v_staged_cols{i}), ...
            'MarkerFaceColor', PlotStyler.colorComponents(v_staged_cols{i}));
    end
    
    Plot_equal = plot(nan, nan, '-o', ...
            'MarkerSize', legend_marker_size, ...
            'Color', PlotStyler.colorComponents(v_anitescu_col), ...
            'MarkerFaceColor', PlotStyler.colorComponents(v_anitescu_col));
    
    Plot_Routh = plot(nan, nan, '-o', ...
        'MarkerSize', legend_marker_size, ...
        'Color', PlotStyler.colorComponents(v_routh_col), ...
        'MarkerFaceColor', PlotStyler.colorComponents(v_routh_end_col), ...
        'MarkerEdgeColor', PlotStyler.colorComponents(v_routh_end_col));
        
    % plot b label
    xlabel(axesLabels{1});
    ylabel(axesLabels{2});
    styler.asCurve();
    legend([Plot_staged{:}, Plot_equal, Plot_Routh], ...
        v_staged_labels{:}, 'Simultaneous', 'Ours', ...
        'Location', 'southwest');
end


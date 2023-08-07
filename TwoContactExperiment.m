function [v_r, v_s] = TwoContactExperiment(fbd, mu_mat,  M_Routh, N_Routh, h, ...
    label_cb, skip_run, skip_plot)

    % extract figure name prefix from parent function
    prefix = dbstack(1).name;
    save_file = [prefix '_data.mat'];
    
    % load data if not re-running
    if nargin < 7
        skip_run = false;
    end
    
    % only run if not plotting
    do_skip_plot = false;
    do_label_cb = label_cb;
    if nargin >= 8
        do_skip_plot = skip_plot;     
    end
    
    if skip_run
        load(save_file);
    else
        tic;
        [v_s, v_a, v_r] = ...
            CompareImpactMethods(fbd, mu_mat, M_Routh, N_Routh, h, false);
        toc
        save(save_file);
    end

    % plot figures
    if ~do_skip_plot
        PlotImpacts(fbd, v_s, v_a, v_r, prefix, do_label_cb);
    end
end
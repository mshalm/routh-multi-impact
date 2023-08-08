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
    if nargin < 8
        skip_plot = false;
    end
    do_label_cb = label_cb;

    if skip_run
        load(save_file);
    else
        tic;
        [v_s, v_a, v_r] = ...
            CompareImpactMethods(fbd, mu_mat, M_Routh, N_Routh, h, false);
        toc
        save(save_file, '-regexp', '^(?!(skip_run|skip_plot|label_cb)$).');
    end

    % plot figures
    if ~skip_plot
        PlotImpacts(fbd, v_s, v_a, v_r, prefix, do_label_cb);
    end
end
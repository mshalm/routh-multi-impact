function PlotImpacts(fbd, v_staged, v_anitescu, ...
    v_Routh, prefix, label_cb)

    % pre-process data
    
    % get contact terms
    [Jn, Jf, pts, bodies] = fbd.contactTerms();
    J = [Jn; Jf];
    m = size(Jn, 1);
    
    % get pre-impact velocity
    vm = fbd.Velocity;
    
    % prepend pre-impact velocity
    v_e = cell(1, size(v_anitescu, 2));
    for i = 1:size(v_anitescu, 2)
        v_e{i} = [vm v_anitescu(:,i)];
    end
    v_anitescu = v_e;
    
    % eliminate staged impacts with null first impact
    alphabet = char('A' + char(0:(m - 1)));
    cell_alphabet = num2cell(alphabet);
    label_suffix = ' first';
    v_staged_labels = strcat(cell_alphabet, label_suffix);
    v_staged_cols = PlotStyler.COLORNAMES(2:(m + 1));
    
    normal_pairs = (0:2:(m-2)) + [1; 2];
    contact_pairs = (1:m) + [m; 0];
    axesList = [normal_pairs contact_pairs];
    axis_types = reshape(repmat({' Normal ',' Tangential '},[m 1]),[2*m 1]);
    cell_alphabet = repmat(cell_alphabet,[1 2])';
    axesLabels = strcat(cell_alphabet, strcat(axis_types, 'Velocity'));
    
    % plot free body diagrams
    
    % label contact points and velocities
    label_cb = @(f) ContactPointLabelCallback(label_cb(f), ...
        bodies, pts, alphabet);
    
    
    % remove extraneous staged impacts
    is_multistep = zeros(length(v_staged), 1);
    for i = 1:length(v_staged)
        is_multistep(i) = size(v_staged{i}, 2) > 2;
    end
    starts_active = ((Jn * vm) < -1e-6);
    use_staged = starts_active & is_multistep;
    
    alphabet = alphabet(use_staged);
    v_staged = v_staged(use_staged);
    v_staged_labels = v_staged_labels(use_staged);
    v_staged_cols = v_staged_cols(use_staged);
    v_anitescu_col = PlotStyler.COLORNAMES{1};
    
    fbd_prefix = [prefix '_diagram'];
    
    % run plots to get bounding box
    PlotDiagrams(fbd, vm, v_staged, v_anitescu, alphabet, ...
        fbd_prefix, label_cb);

    % plot impact trajectories
    for i = 1:size(axesList, 2)
        %tic;
        Jax = axesList(:, i)';
        styler = PlotImpactTrajectory(J, v_staged, v_staged_labels, ...
            v_staged_cols, v_anitescu, v_anitescu_col, v_Routh, Jax, ...
            axesLabels);
        fname = [prefix '_J' int2str(Jax(1)) '_' int2str(Jax(2))];
        styler.print(fname).close();
        %toc
    end
    
end

function fbd = ContactPointLabelCallback(fbd, bodies, points, alphabet)
    for i = 1:numel(alphabet)
        fbd = fbd.labelPoint(bodies{i}, points{i}, '', 'r', 0, 't');
    end
    for i = 1:numel(alphabet)
        %label_i = ['$\boldsymbol{' alphabet(i) '}$'];
        label_i = alphabet(i);
        fbd = fbd.labelVelocity(bodies{i}, points{i}, label_i, 'r');
    end
end

function limits = PlotDiagrams(fbd, vm, v_staged, v_anitescu, alphabet, ...
    fbd_prefix, label_cb)
    
    % Run once to get limits. Then, run again to set them.
    lim_set = {};
    for do_save = [false true]
        
        % restore original velocity
        fbd.Velocity = vm;
        
        limits = zeros(0, 4);
        fbd = fbd.view(label_cb, lim_set{:});
        limits = [limits; xlim(gca) ylim(gca)];
        if do_save
            fbd.Styler.print([fbd_prefix '_vm']);
        end
        fbd.Styler.close();

        for i = 1:length(v_staged)
            for j = 2:size(v_staged{i}, 2)
                filename = ...
                    [fbd_prefix '_' alphabet(i) '_' int2str(j)];
                fbd.Velocity = v_staged{i}(:, j);
                fbd = fbd.view(label_cb, lim_set{:});
                limits = [limits; xlim(gca) ylim(gca)];
                if do_save
                    fbd.Styler.print(filename);
                end
                fbd.Styler.close();
            end
        end

        for i = 1:length(v_anitescu)
            filename = [fbd_prefix '_Simultaneous_' int2str(i)];
            fbd.Velocity = v_anitescu{i}(:, 2);
            fbd = fbd.view(label_cb, lim_set{:});
            limits = [limits; xlim(gca) ylim(gca)];
            if do_save
                fbd.Styler.print(filename);
            end
            fbd.Styler.close();
        end
        limits = [min(limits(:,1)), max(limits(:,2)), ...
                  min(limits(:,3)), max(limits(:,4))];
        lim_set = {limits};
    end
    
    % restore original velocity
    fbd.Velocity = vm;
end

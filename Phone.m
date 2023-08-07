function [v_r, fbd, v_s] = Phone(varargin)
    % dimensions
    % width, height, mass, friction, drop velocity
    [a, b, m, mu, v0] = PhoneParameters();
    
    mu_mat = diag([mu mu]);
    
    % sim params
    h = ceil((m * v0 / 2) * 10) / 10;
    N_Routh = 10;
    M_Routh = 2 ^ 14;
    
    % construct system
    fbd = FreeBodyDiagram();
    
    % rimless wheel
    fbd = fbd.addFrame('base', 'object', 'floating');
    fbd = fbd.addBox('object', 'phone', a, b, m, 'y');
    
    % floor
    fbd = fbd.addWall('floor', [0 1], 0, 'l2');
    
    % construct initial condition
    fbd.Configuration = [0 b/2 0]';
    fbd.Velocity = [0 -v0 0]';
    
    % set up labelling callback
    label_callback = @(f, s) f.labelVelocity('phone', [0 0]', v_of_stage(s), 'r');
    
    % run experiment
    [v_r, v_s] = TwoContactExperiment(fbd, mu_mat, M_Routh, N_Routh, h, ...
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

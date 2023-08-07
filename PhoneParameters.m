function [a, b, m, mu, v0] = PhoneParameters
    % width
    %a = .07444; 
    a = 1;
    % height
    b = 2;
    %b = .16094;
    
    % central mass
    %m = .190;
    m = 1;
    % friction
    mu = 1;
    
    % drop height
    %y_init = 0.001;
    y_init = .01;
    v0 = sqrt(2 * 9.81 * y_init);
end


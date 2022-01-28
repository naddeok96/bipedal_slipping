function [q, dq, ddq] = angle_constraints(q, dq, ddq)
    % Input
    % ~ q, dq, ddq
    
    %% Constraints from "Range of motion measurements: reference values and a database for comparison studies"
    %   -- J. M. SOUCIE, C. WANG, A. FORSYTH, S. FUNK, M. DENNY, K. E. ROACH,D. BOONE
    hip_flexion_limit           = deg2rad(140); % Raise leg  towards chest
    hip_extension_limit         = deg2rad( 29); % Pull  leg  towards glute
    knee_flexion_limit          = deg2rad(150); % Pull  foot towards glute
    knee_extension_limit        = deg2rad(  5); % Raise foot towards chest
    ankle_dorsiflexion_limit    = deg2rad( 25); % Raise toes towards chest
    ankle_plantar_flexion_limit = deg2rad( 67); % Pull  toes towards glute
    
%     q(1) + q(2) + q(4) + q(6) % Free ankle
%     q(1) + q(2) + q(4) % Free knee
%     q(1) + q(2) % Hip
%     q(1) % Stance knee
%     q(1) + q(3) % Stance ankle
%     q(1)+q(7) % HAT
    
    %% Bound Angles
    bound(1,2,3);
    stop = ones(7, 1);
    [q(1), stop(1)] = bound( q(1)        , -hip_extension_limit        , hip_flexion_limit);        % Stance Thigh
    [q(2), stop(2)] = bound( q(2) + q(1) , -hip_extension_limit        , hip_flexion_limit);        % Free Thigh
    q(2) = q(2) - q(1);
    [q(3), stop(3)] = bound( q(3)        , -knee_flexion_limit         , knee_extension_limit);     % Stance Shank
    [q(4), stop(4)] = bound( q(4)        , -knee_flexion_limit         , knee_extension_limit);     % Free Shank
    [q(5), stop(5)] = bound( q(5) - pi/2 , -ankle_plantar_flexion_limit, ankle_dorsiflexion_limit); % Stance Foot
    q(5) = q(5) + pi/2;
    [q(6), stop(6)] = bound( q(6) - pi/2 , -ankle_plantar_flexion_limit, ankle_dorsiflexion_limit); % Free Foot
    q(6) = q(6) + pi/2;
    [q(7), stop(7)] = bound( q(7) + q(1) , pi/2                        , 3*pi/2);                   % HAT
    q(7) = q(7) - q(1);
    
    dq  = stop.*dq;
    ddq = stop.*ddq;
    
    %% Bounding Function
    function [y, stop] = bound(x, low, up)
       % Input
       % ~ x   : value to bound
       % ~ low : lower bound
       % ~ up  : upper bound
       % Output
       % ~ y   : bounded input value
       stop = 1;
       
       if up < x || x < low
           stop = 0;
       end
       
       y = min(max(x, low), up); 
    end
    
end

function update_plot(q, model_params, ST, SA, SK, H, FK, FA, FT, torso_end,stancefoot_line, freefoot_line, stanceThigh_line, stanceShank_line, freeThigh_line, freeShank_line, torso_line, body_line)
   
    % Clear points
    clearpoints(stancefoot_line)
    clearpoints(freefoot_line)
    clearpoints(stanceThigh_line)
    clearpoints(stanceShank_line)
    clearpoints(freeThigh_line)
    clearpoints(freeShank_line)
    clearpoints(torso_line)
    clearpoints(body_line)
    
    % Geometric params
    gamma = model_params.gamma;
    R     = model_params.R;
    xf    = model_params.xf;

    % Draw feet
    stance_abs_angle = q(1) + q(3) + q(5) + gamma + pi/2;
    [stance_x, stance_y, ~] = draw_foot(R ,ST ,xf, stance_abs_angle, q(5), pi/2 + q(5), pi/2 - q(5));
    
    free_abs_angle = q(1) + q(2) + q(4) + q(6) + gamma + pi/2;
    [free_x, free_y, ~] = draw_foot(R ,FT ,xf, free_abs_angle, q(6), pi/2 + q(6), pi/2 - q(6));
    
    % Plot feet
    for i = 1:length(stance_x)
       addpoints(stancefoot_line, stance_x(i), stance_y(i))
       addpoints(freefoot_line,     free_x(i),   free_y(i))
    end
    
    % Plot limbs
    addpoints(stanceThigh_line, [ H(1) SK(1)], [ H(2) SK(2)])
    addpoints(stanceShank_line, [SK(1) SA(1)], [SK(2) SA(2)])
    addpoints(freeThigh_line,   [ H(1) FK(1)], [ H(2) FK(2)])
    addpoints(freeShank_line,   [FK(1) FA(1)], [FK(2) FA(2)])
    
    addpoints(torso_line,       [H(1)  torso_end(1)],[H(2) torso_end(2)])
    addpoints(body_line, H(1), H(2))
    axis([-0.25*model_params.height 4*model_params.height -0.1*model_params.height model_params.height])
    drawnow

end

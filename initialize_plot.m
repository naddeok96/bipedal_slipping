function [stancefoot_line,freefoot_line,stanceThigh_line,stanceShank_line,freeThigh_line,freeShank_line,torso_line,body_line] = initialize_plot(q, model_params, ST, SA, SK, H, FK, FA, FT, torso_end)
 
    % Set colors
    stanceFoot_color  = [1    0.39 0.28]; % tomato red
    freeFoot_color    = [0.28 0.46 1];    % cornflower blue
    stanceShank_color = [1    0    0];    % red
    freeShank_color   = [0    0    1];    % blue
    stanceThigh_color = [0.75 0    0];    % dark red
    freeThigh_color   = [0.10 0.25 0.75]; % dark blue
    body_color        = [0.50 0    1];    % purple
    torso_color       = [0.50 0    1];    % purple

    % Geometric params
    groundLength = 2;
    gamma = model_params.gamma;
    R     = model_params.R;
    xf    = model_params.xf;
    
    % Set Floor
    scatter(0,0, 5)
    plot([-3*groundLength * cos(gamma); 3*groundLength * cos(gamma)],[0; 3*groundLength*sin(gamma)],'k');
    axis([-0.25*model_params.height 4*model_params.height -0.1*model_params.height model_params.height])
    hold on
    
    % Draw feet
    stance_abs_angle = q(1) + q(3) + q(5) + gamma + pi/2;
    [stance_x, stance_y, ~] = draw_foot(R ,ST ,xf, stance_abs_angle, q(5), pi/2 + q(5), pi/2 - q(5));
    
    free_abs_angle = q(1) + q(2) + q(4) + q(6) + gamma + pi/2;
    [free_x, free_y, ~] = draw_foot(R ,FT ,xf, free_abs_angle, q(6), pi/2 + q(6), pi/2 - q(6));
    
    % Plot feet
    stancefoot_line = animatedline('LineWidth', 2, 'Color', stanceFoot_color);
    freefoot_line   = animatedline('LineWidth', 2, 'Color', freeFoot_color);
    for i = 1:length(stance_x)
       addpoints(stancefoot_line, stance_x(i), stance_y(i))
       addpoints(freefoot_line,     free_x(i),   free_y(i))
    end
    
    % Plot limbs
    stanceThigh_line = animatedline([H(1)  SK(1)],  [H(2)  SK(2)], 'LineWidth',2,'Color',stanceThigh_color);
    stanceShank_line = animatedline([SK(1) SA(1)], [SK(2) SA(2)],'LineWidth',2,'Color',stanceShank_color);
    freeThigh_line   = animatedline([H(1)  FK(1)],  [H(2)  FK(2)], 'LineWidth',2,'Color',freeThigh_color);
    freeShank_line   = animatedline([FK(1) FA(1)], [FK(2) FA(2)],'LineWidth',2,'Color',freeShank_color);
    torso_line       = animatedline([H(1)  torso_end(1)],[H(2) torso_end(2)],'LineWidth',2,'Color',torso_color);
    
    % Plot hip
    body_line = animatedline('Marker', 'o', 'Color',body_color, 'MarkerFaceColor',body_color, 'MarkerSize',15);
    addpoints(body_line, H(1), H(2))
    drawnow

end

function next_step(~,~,gui,dt,model_params,ctrl_params,SC,ST,SA,SK,H,FK,FA,FT,FC,torso_end,torso_com, stancefoot_line, freefoot_line, stanceThigh_line, stanceShank_line, freeThigh_line, freeShank_line, torso_line, body_line)
    % Load values
    q   = cell2mat(gui.config_table.Data(:,2));
    dq  = cell2mat(gui.config_table.Data(:,3));
    u   = cell2mat(gui.config_table.Data(:,5));
    
    % Calculate matricies   
    [~, D] = get_De_7links_sgl(q, model_params);             % Inertia
    [~, G] = get_Ge_7links_sgl(q, model_params);             % Gravity
    [~, C] = get_Ce_7links_sgl(q, [dq; 0; 0], model_params); % Coriolis and Centripetal
                                            
    % Calculate accelerations (ddq) Eq. 3 in Martin & Schmiedeler : Predicting Human Walking Gaits with a Simple Planar Model)
    ddq = D \ ((ctrl_params.B * u) - (C * dq) - G);
    
    % Calculate angle and velocity with kinematics
    q0 = q;
    q  = q0 +  dq*dt + 0.5*ddq*dt^2;
    dq = dq + ddq*dt; 
    
    % Constrain
    [q, dq, ddq] = angle_constraints(q, dq, ddq);
      
    
    % Get initial and terminal conditions
    [foot_height, step_length] = swing_foot_position(q(1:6), q(1:6), model_params);
    
    if step_length < 0
        disp('No forward progess.') 
    elseif foot_height < 0
       disp('Foot has made impact.')
       [~, f] = impact_event7links([q;dq], model_params);
       sprintf('Vector of horizontal and vertical impulsive forces [%0.2f, %0.2f]  Newtons',f(1),f(2))
        
        [SC, ST, SA, SK, FK, FA, FT, FC] = switch_stance_cart_coordinates(SC, ST, SA, SK, FK, FA, FT, FC);
        SC(2) = 0;
        
        [q0,q,dq,ddq] = switch_stance_joints(q0,q,dq,ddq);
       
    end
    
    % Update coordinates
    [~, ST, SA, SK, H, FK, FA, FT, ~, torso_end, ~] = next_cart_coordinates(q, q0, SC, model_params, ST, SA, SK, H, FK, FA, FT, FC, torso_end, torso_com);

    update_plot(q, model_params, ST, SA, SK, H, FK, FA, FT, torso_end, stancefoot_line, freefoot_line, stanceThigh_line, stanceShank_line, freeThigh_line, freeShank_line, torso_line, body_line)
    update_gui(gui,q0, q, dq, ddq, u)
    
    
end
close all; clc; clear; format compact;
folder = fileparts(which(mfilename)); 
addpath(genpath(folder))

%% Global variables
global q0 q dq ddq u dt
global model_params ctrl_params
global SC ST SA SK H FK FA FT FC torso_end torso_com

%% Hyperparameters
% Physical
height = 1.8288;
mass = 75;

% Calculate Model Parameters based off height and mass
model_params = get_model_params(height, mass);
ctrl_params  = get_ctrl_params();

% Inital configuration
dt = 1e-2; 

% q = [ % Beginning of step
%      0.4709;    % 1 Free Thigh
%      -0.7512;   % 2 Between Thighs
%      -0.2584 ;  % 3 Free Shank
%      -0.3531;   % 4 Back Shank
%      1.8002;    % 5 Free Foot
%      1.5495;    % 6 Stance Foot
%      13/16 * pi % 7 HAT
%      ];

% % q = [ % End of step
% %      0.6;    % 1 Free Thigh
% %      -0.7512;   % 2 Between Thighs
% %      -0.2584 ;  % 3 Stance Shank
% %      -0.3531;   % 4 Free Shank
% %      1.8002;    % 5 Stance Foot
% %      1.5495;    % 6 Free Foot
% %      13/16 * pi % 7 HAT
% %      ];
 
[q0, dq, ddq] = deal(zeros(7,1));

u = zeros(6,1);
 
%% Initialize coordinates
%   ~ SC: stance contact
%   ~ ST: stance foot center of curvature
%   ~ SA: stance ankle
%   ~ SK: stance knee
%   ~ H:  hip
%   ~ FK: free knee
%   ~ FA: free ankle
%   ~ FT: free foot center of curvature
%   ~ FC: lowest point on free foot
[SC, ST, SA, SK, H, FK, FA, FT, FC, torso_end, torso_com] = get_cart_coordinates(q, model_params);

% Switch Foot
[SC, ST, SA, SK, FK, FA, FT, FC] = switch_stance_cart_coordinates(SC, ST, SA, SK, FK, FA, FT, FC);
S  = [
    1  1 0 0 0 0 0
    0 -1 0 0 0 0 0
    0  0 0 1 0 0 0
    0  0 1 0 0 0 0
    0  0 0 0 0 1 0
    0  0 0 0 1 0 0
    0 -1 0 0 0 0 1];
q = S*q;

% [SC, ST, SA, SK, FK, FA, FT, FC] = check_coordinates(SC, ST, SA, SK, FK, FA, FT, FC);

%% Display
% Initialize GUI with buttons
initialize_gui(q0, q, dq, ddq, u);
initialize_plot(q, model_params, ...
                ST, SA, SK, H, FK, FA, FT, torso_end)

%% Functions
function initialize_gui(q0, q, dq, ddq, u)
    global gui
    
    %% Initialize Plot
    gui.fig = figure;
    set(gcf, 'units','normalized','outerposition',[0 0 1 1])  
    axis equal
    axis off
    hold on
    
    %% Add Table
    table_left   = 0.6;
    table_bottom = 0.6;
    table_width  = 0.34;
    table_height = 0.2175;
    locations = [ "1) Free Thigh", "2) Between Thighs", "3) Free Shank", "4) Back Shank", "5) Free Foot", "6) Stance Foot", "7) HAT"]';
    ue = [u(1:5); 0; u(6)];
    table_data = table(q0, q, dq, ddq, ue);
    vars = {'q0','q', 'dq', 'ddq', 'u'};
    gui.config_table = uitable(gui.fig, 'Data', table2cell(table_data),... 
                            'RowName', locations, 'ColumnName', vars,...
                            'Units','Normalized', 'Position', [table_left table_bottom table_width table_height]);
    table_extent = get(gui.config_table,'Extent');
    set(gui.config_table,'Position',[table_left table_bottom table_extent(3) table_extent(4)])
    
    %% Apply Torque Button
    pb_left   = 0.67;
    pb_bottom = 0.09;
    pb_width  = 0.2;
    pb_height = 0.1;
    gui.push_button = uicontrol('style','push',...
                     'units','normalized',...
                     'position',[pb_left pb_bottom pb_width pb_height],...
                     'fontsize',14,...
                     'string','Apply Torque',...
                     'callback',{@applyTorque, gui});

    %% Sliders
    slider_left   = 0.65;
    slider_bottom = 0.49;
    slider_height = 0.05;
    slider_width  = 0.25;
    
    min_val = -100;
    max_val =  100;

    % U2 Slider
    gui.slider_u2 = uicontrol('style','slider',...
                             'units','normalized',...
                             'position',[slider_left slider_bottom slider_width slider_height],...
                             'fontsize',14,...
                             'BackgroundColor', 'w', ...
                             'Value',u(1), ...
                             'Min', min_val,'Max', max_val,...
                             'callback', {@getSliderValues, gui});    
    set(gui.slider_u2, 'Units', 'normalized')

    % U3 Slider
    gui.slider_u3 = uicontrol('style','slider',...
                             'units','normalized',...
                             'position',[slider_left slider_bottom-slider_height slider_width slider_height],...
                             'fontsize',14,...
                             'BackgroundColor', 'w', ...
                             'Value',u(2), ...
                             'Min', min_val,'Max', max_val,...
                             'callback', {@getSliderValues, gui});    
    set(gui.slider_u3, 'Units', 'normalized')

    % U4 Slider
    gui.slider_u4 = uicontrol('style','slider',...
                             'units','normalized',...
                             'position',[slider_left slider_bottom-2*slider_height slider_width slider_height],...
                             'fontsize',14,...
                             'BackgroundColor', 'w', ...
                             'Value',u(3), ...
                             'Min', min_val,'Max', max_val,...
                             'callback', {@getSliderValues, gui});    
    set(gui.slider_u4, 'Units', 'normalized')

    % U5 Slider
    gui.slider_u5 = uicontrol('style','slider',...
                             'units','normalized',...
                             'position',[slider_left slider_bottom-3*slider_height slider_width slider_height],...
                             'fontsize',14,...
                             'BackgroundColor', 'w', ...
                             'Value',u(4), ...
                             'Min', min_val,'Max', max_val,...
                             'callback', {@getSliderValues, gui});    
    set(gui.slider_u5, 'Units', 'normalized')

    % U6 Slider
    gui.slider_u6 = uicontrol('style','slider',...
                             'units','normalized',...
                             'position',[slider_left slider_bottom-4*slider_height slider_width slider_height],...
                             'fontsize',14,...
                             'BackgroundColor', 'w', ...
                             'Value',u(5), ...
                             'Min', min_val,'Max', max_val,...
                             'callback', {@getSliderValues, gui});    
    set(gui.slider_u6, 'Units', 'normalized')

    % U2 Slider
    gui.slider_u7 = uicontrol('style','slider',...
                             'units','normalized',...
                             'position',[slider_left slider_bottom-5*slider_height slider_width slider_height],...
                             'fontsize',14,...
                             'BackgroundColor', 'w', ...
                             'Value',u(6), ...
                             'Min', min_val,'Max', max_val,...
                             'callback', {@getSliderValues, gui});    
    set(gui.slider_u7, 'Units', 'normalized')

end

function [x, y, AA] = draw_foot(r,center,xf,abs_angle,foot_angle,phi_b,phi_t)
    % given a radius, center point and abs shank angle, draws a foot
    % Inputs:
    %  ~ r: the radius of the circle
    %  ~ center: the coordinates of the center point of the circle, pass in as
    %    a vector
    %  ~ abs_angle: the abs angle of the foot at given moment 
    %  ~ foot_angle: the angle of the foot relative to the shank (q5 or q6)

    

    % Plot the foot
    theta = linspace(abs_angle - phi_b - pi, abs_angle + phi_t - pi);
    x = center(1)+r.*cos(theta);
    y = center(2)+r.*sin(theta);
    x(end+1) = x(1);
    y(end+1) = y(1);

    % Determine where the the foot and shank intersect
    xAA = -xf*sin(foot_angle)*(sin(phi_b)+sin(phi_t))/(cos(phi_b-foot_angle)-cos(foot_angle+phi_t)) + r*sin(phi_b+phi_t)*cos(foot_angle)/(cos(phi_b-foot_angle)-cos(foot_angle+phi_t));
    yAA = xAA*(cos(phi_b)-cos(phi_t))/(sin(phi_b)+sin(phi_t)) - r*sin(phi_b+phi_t)/(sin(phi_b)+sin(phi_t));
    AA = [
         xAA*sin(abs_angle) + yAA*cos(abs_angle) + center(1)
        -xAA*cos(abs_angle) + yAA*sin(abs_angle) + center(2)];
end

function initialize_plot(q, model_params, ST, SA, SK, H, FK, FA, FT, torso_end)
    global stancefoot_line freefoot_line stanceThigh_line stanceShank_line freeThigh_line freeShank_line torso_line body_line
    
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

function update_plot(q, model_params, ST, SA, SK, H, FK, FA, FT, torso_end)
    global stancefoot_line freefoot_line stanceThigh_line stanceShank_line freeThigh_line freeShank_line torso_line body_line
    
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

function getSliderValues(varargin)
    global gui u q0 q dq ddq
    
    % Gather u from slider
    u(1) = gui.slider_u2.Value;
    u(2) = gui.slider_u3.Value;
    u(3) = gui.slider_u4.Value;
    u(4) = gui.slider_u5.Value;
    u(5) = gui.slider_u6.Value;
    u(6) = gui.slider_u7.Value;
    
    % Update text
    update_gui(q0, q, dq, ddq, u)
    
end

function applyTorque(varargin)
    global q0 q dq ddq u dt
    global model_params ctrl_params
    global SC ST SA SK H FK FA FT FC torso_end torso_com
    
    % Calculate matricies   
    [~, D] = get_De_7links_sgl(q, model_params);             % Inertia
    [~, G] = get_Ge_7links_sgl(q, model_params);             % Gravity
    [~, C] = get_Ce_7links_sgl(q, [dq; 0; 0], model_params); % Coriolis and Centripetal
                                            
    % Calculate accelerations (ddq) Eq. 3 in Martin & Schmiedeler : Predicting Human Walking Gaits with a Simple Planar Model)
    ddq = D \ ((ctrl_params.B * u) - (C * dq) - G);
    
    % Calculate angle and velocity with kinematics
    q0 = q;
    q  = q0 +  dq*dt + 0.5*ddq*dt^2;
    [q, dq, ddq] = angle_constraints(q, dq, ddq);
    dq = dq + ddq*dt;   
    
    % Get initial and terminal conditions
    [foot_height, step_length] = swing_foot_position(q(1:6), q(1:6), model_params);
    
    if step_length < 0
        disp('No forward progess.') 
    elseif foot_height < 0
       disp('Foot has made impact.')
    end
    
    % Update coordinates, plot and 
    %   ~ SC: stance contact
    %   ~ ST: stance foot center of curvature
    %   ~ SA: stance ankle
    %   ~ SK: stance knee
    %   ~ H:  hip
    %   ~ FK: free knee
    %   ~ FA: free ankle
    %   ~ FT: free foot center of curvature
    %   ~ FC: lowest point on free foot
    [SC, ST, SA, SK, H, FK, FA, FT, FC, torso_end, torso_com] = next_cart_coordinates(q, q0, SC, model_params, ST, SA, SK, H, FK, FA, FT, FC, torso_end, torso_com);
%     [SC, ST, SA, SK, FK, FA, FT, FC]    = check_coordinates(SC, ST, SA, SK, FK, FA, FT, FC);


    update_plot(q, model_params, ST, SA, SK, H, FK, FA, FT, torso_end)
    update_gui(q0, q, dq, ddq, u)
    
end

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

function update_gui(q0, q, dq, ddq, u)
    global gui 
    ue = [u(1:5); 0; u(6)];
    gui.config_table.Data =  table2cell(table(q0, q,dq, ddq, ue));
    
end

function fig_points()
%% Stick Figure Points
% Points at joints with "listeners" for events when joint positions are adjusted
% toe_t_point = images.roi.Point(gca,'Position', toe_t', 'MarkerSize', 2);
% addlistener(toe_t_point,'MovingROI',@toe_t_change);
% addlistener(toe_t_point,'ROIMoved', @toe_t_change);
% 
% toe_l_point = images.roi.Point(gca,'Position', toe_l', 'MarkerSize', 2);
% addlistener(toe_l_point,'MovingROI',@toe_l_change);
% addlistener(toe_l_point,'ROIMoved', @toe_l_change);
% 
% kneck_point = images.roi.Point(gca,'Position', kneck', 'MarkerSize', 2);
% addlistener(kneck_point,'MovingROI',@kneck_change);
% addlistener(kneck_point,'ROIMoved',@kneck_change);
% 
% knee_t_point = images.roi.Point(gca,'Position', knee_t', 'MarkerSize', 2);
% addlistener(knee_t_point,'MovingROI',@knee_t_change);
% addlistener(knee_t_point,'ROIMoved',@knee_t_change);
% 
% knee_l_point = images.roi.Point(gca,'Position', knee_l', 'MarkerSize', 2);
% addlistener(knee_l_point,'MovingROI',@knee_l_change);
% addlistener(knee_l_point,'ROIMoved',@knee_l_change);
% 
% ankle_t_point = images.roi.Point(gca,'Position', ankle_t', 'MarkerSize', 2);
% addlistener(ankle_t_point,'MovingROI',@ankle_t_change);
% addlistener(ankle_t_point,'ROIMoved', @ankle_t_change);
% 
% ankle_l_point = images.roi.Point(gca,'Position', ankle_l', 'MarkerSize', 2);
% addlistener(ankle_l_point,'MovingROI',@ankle_l_change);
% addlistener(ankle_l_point,'ROIMoved', @ankle_l_change);
end

function kneck_change(~, evt)
global q hip kneck kneck_point torso_line 
if evt.EventName == "ROIMoved"
    kneck_point.Position = kneck';
    
else
    v1 = evt.PreviousPosition  - hip';
    v2  = evt.CurrentPosition  - hip';

    angle = atan2d( det([v1;v2;]) , dot(v1,v2) );

    q(7) = q(7) + angle;

    update_coordinates()

    clearpoints(torso_line)
    addpoints(torso_line ,[ hip(1)     kneck(1)],  [hip(2)     kneck(2)]);  
    drawnow
end
end

function knee_t_change(~, evt)
global q 
global hip knee_t ankle_t toe_t heel_t foot_curve_t
global ankle_t_point knee_t_point toe_t_point  
global thigh_t_line shank_t_line toe_t_line heel_t_line foot_curve_t_line

if evt.EventName == "ROIMoved"
    knee_t_point.Position = knee_t';
       
    update_floor()
    
else
    v1 = evt.PreviousPosition  - hip';
    v2  = evt.CurrentPosition  - hip';

    angle = atan2d( det([v1;v2;]) , dot(v1,v2) );

    q(2) = q(2) + angle;

    update_coordinates()

    clearpoints(toe_t_line)
    clearpoints(heel_t_line)
    clearpoints(foot_curve_t_line)
    clearpoints(thigh_t_line)
    clearpoints(shank_t_line)
    addpoints(toe_t_line, [ ankle_t(1)  toe_t(1)],  [ankle_t(2)  toe_t(2)])
    addpoints(heel_t_line, [ ankle_t(1)  heel_t(1)],  [ankle_t(2)  heel_t(2)])
    addpoints(foot_curve_t_line, foot_curve_t(:, 1), foot_curve_t(:, 2))
    addpoints(thigh_t_line,[ hip(1)  knee_t(1)],  [hip(2)  knee_t(2)]);  
    addpoints(shank_t_line,[ knee_t(1) ankle_t(1)], [knee_t(2) ankle_t(2)]);
    drawnow
end

ankle_t_point.Position = ankle_t';
toe_t_point.Position = toe_t';
end

function knee_l_change(~, evt)
global q 
global hip knee_l ankle_l toe_l heel_l foot_curve_l
global ankle_l_point knee_l_point toe_l_point 
global thigh_l_line shank_l_line toe_l_line heel_l_line foot_curve_l_line

if evt.EventName == "ROIMoved"
    knee_l_point.Position = knee_l';
    
    update_floor()
else
    v1 = evt.PreviousPosition  - hip';
    v2  = evt.CurrentPosition  - hip';

    angle = atan2d( det([v1;v2;]) , dot(v1,v2) );

    q(1) = q(1) + angle;

    update_coordinates()

    clearpoints(toe_l_line)
    clearpoints(heel_l_line)
    clearpoints(foot_curve_l_line)
    clearpoints(thigh_l_line)
    clearpoints(shank_l_line)
    addpoints(toe_l_line, [ ankle_l(1)  toe_l(1)],  [ankle_l(2)  toe_l(2)])
    addpoints(heel_l_line, [ ankle_l(1)  heel_l(1)],  [ankle_l(2)  heel_l(2)])
    addpoints(foot_curve_l_line, foot_curve_l(:, 1), foot_curve_l(:, 2))
    addpoints(thigh_l_line,[ hip(1)  knee_l(1)],  [hip(2)  knee_l(2)]);  
    addpoints(shank_l_line,[ knee_l(1) ankle_l(1)], [knee_l(2) ankle_l(2)]);
    drawnow
end
ankle_l_point.Position = ankle_l';
toe_l_point.Position = toe_l';
end

function ankle_t_change(~, evt)
global q 
global knee_t ankle_t toe_t heel_t foot_curve_t
global ankle_t_point toe_t_point 
global shank_t_line toe_t_line heel_t_line foot_curve_t_line

if evt.EventName == "ROIMoved"
    ankle_t_point.Position = ankle_t';
    
    update_floor()
else
    v1 = evt.PreviousPosition  - knee_t';
    v2  = evt.CurrentPosition  - knee_t';

    angle = atan2d( det([v1;v2;]) , dot(v1,v2) );

    q(4) = q(4) + angle;

    update_coordinates()

    clearpoints(shank_t_line)
    clearpoints(toe_t_line)
    clearpoints(heel_t_line)
    clearpoints(foot_curve_t_line)
    addpoints(shank_t_line,[ knee_t(1) ankle_t(1)], [knee_t(2) ankle_t(2)]);
    addpoints(toe_t_line,  [ ankle_t(1) toe_t(1)], [ankle_t(2) toe_t(2)]);
    addpoints(heel_t_line, [ ankle_t(1) heel_t(1)], [ankle_t(2) heel_t(2)]);
    addpoints(foot_curve_t_line,foot_curve_t(:,1), foot_curve_t(:,2));
    drawnow
end
toe_t_point.Position = toe_t';
end

function ankle_l_change(~, evt)
global q 
global knee_l ankle_l toe_l heel_l foot_curve_l
global ankle_l_point toe_l_point 
global shank_l_line toe_l_line heel_l_line foot_curve_l_line

if evt.EventName == "ROIMoved"
    ankle_l_point.Position = ankle_l';
    
    update_floor()
else
    v1 = evt.PreviousPosition  - knee_l';
    v2  = evt.CurrentPosition  - knee_l';

    angle = atan2d( det([v1;v2;]) , dot(v1,v2) );

    q(3) = q(3) + angle;

    update_coordinates()

    clearpoints(shank_l_line)
    clearpoints(toe_l_line)
    clearpoints(heel_l_line)
    clearpoints(foot_curve_l_line)
    addpoints(shank_l_line,[ knee_l(1) ankle_l(1)], [knee_l(2) ankle_l(2)]);
    addpoints(toe_l_line,  [ ankle_l(1) toe_l(1)], [ankle_l(2) toe_l(2)]);
    addpoints(heel_l_line, [ ankle_l(1) heel_l(1)], [ankle_l(2) heel_l(2)]);
    addpoints(foot_curve_l_line,foot_curve_l(:,1), foot_curve_l(:,2));
    drawnow
end
toe_l_point.Position = toe_l';
end

function toe_l_change(~, evt)
global q 
global ankle_l toe_l heel_l foot_curve_l
global toe_l_point 
global toe_l_line heel_l_line foot_curve_l_line

if evt.EventName == "ROIMoved"
    toe_l_point.Position = toe_l';
    
    update_floor()
else
    v1 = evt.PreviousPosition  - toe_l';
    v2  = evt.CurrentPosition  - toe_l';

    angle = atan2d( det([v1;v2;]) , dot(v1,v2) );

    q(5) = q(5) + angle;

    update_coordinates()

    clearpoints(toe_l_line)
    clearpoints(heel_l_line)
    clearpoints(foot_curve_l_line)
    addpoints(toe_l_line,  [ ankle_l(1) toe_l(1)], [ankle_l(2) toe_l(2)]);
    addpoints(heel_l_line, [ ankle_l(1) heel_l(1)], [ankle_l(2) heel_l(2)]);
    addpoints(foot_curve_l_line,foot_curve_l(:,1), foot_curve_l(:,2));
    drawnow
end
end

function toe_t_change(~, evt)
global q 
global ankle_t toe_t heel_t foot_curve_t
global toe_t_point
global toe_t_line heel_t_line foot_curve_t_line

if evt.EventName == "ROIMoved"
    toe_t_point.Position = toe_t';
    
    update_floor()
else
    v1 = evt.PreviousPosition  - toe_t';
    v2  = evt.CurrentPosition  - toe_t';

    angle = atan2d( det([v1;v2;]) , dot(v1,v2) );

    q(6) = q(6) + angle;

    update_coordinates()

    clearpoints(toe_t_line)
    clearpoints(heel_t_line)
    clearpoints(foot_curve_t_line)
    addpoints(toe_t_line,  [ ankle_t(1) toe_t(1)], [ankle_t(2) toe_t(2)]);
    addpoints(heel_t_line, [ ankle_t(1) heel_t(1)], [ankle_t(2) heel_t(2)]);
    addpoints(foot_curve_t_line,foot_curve_t(:,1), foot_curve_t(:,2));
    drawnow
end
end


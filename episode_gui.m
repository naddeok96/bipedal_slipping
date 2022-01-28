close all; clc; clear; format compact;
folder = fileparts(which(mfilename)); 
addpath(genpath(folder))

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

q = [ % End of step
     0.60;    % 1 Free Thigh
     -0.7512;   % 2 Between Thighs
     -0.2584 ;  % 3 Stance Shank
     -0.3531;   % 4 Free Shank
     1.8002;    % 5 Stance Foot
     1;    % 6 Free Foot
     13/16 * pi % 7 HAT
     ];
 
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

%% Initialize GUI with buttons
initialize_gui(q0,q,dq,ddq,u,dt,model_params,ctrl_params,SC,ST,SA,SK,H,FK,FA,FT,FC,torso_end,torso_com);

%% Functions
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


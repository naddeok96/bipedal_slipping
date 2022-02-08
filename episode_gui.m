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
 
[q0, dq, ddq, u] = deal(zeros(7,1));

 
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

%% Initialize figure
gui.fig = figure;
set(gcf, 'units','normalized','outerposition',[0 0 1 1])  
axis equal
axis off
hold on

%% Add plot
[stancefoot_line,freefoot_line,stanceThigh_line,stanceShank_line,freeThigh_line,freeShank_line,torso_line,body_line] = initialize_plot(q, model_params,ST, SA, SK, H, FK, FA, FT, torso_end);

%% Add table
table_left   = 0.6;
table_bottom = 0.6;
table_width  = 0.34;
table_height = 0.2175;
locations = [ "1) Free Thigh", "2) Between Thighs", "3) Free Shank", "4) Back Shank", "5) Free Foot", "6) Stance Foot", "7) HAT"]';
table_data = table(q0, q, dq, ddq, u);
vars = {'q0','q', 'dq', 'ddq', 'u'};
gui.config_table = uitable(gui.fig, 'Data', table2cell(table_data),... 
                        'RowName', locations, 'ColumnName', vars,...
                        'Units','Normalized', 'Position', [table_left table_bottom table_width table_height]);
table_extent = get(gui.config_table,'Extent');
set(gui.config_table,'Position',[table_left table_bottom table_extent(3) table_extent(4)])

%% Add sliders
slider_left   = 0.65;
slider_bottom = 0.49;
slider_height = 0.05;
slider_width  = 0.25;

min_val = -1000;
max_val =  1000;

% U1 Slider
gui.slider_u1 = uicontrol('style','slider',...
                         'string','1',...
                         'units','normalized',...
                         'position',[slider_left slider_bottom slider_width slider_height],...
                         'fontsize',14,...
                         'BackgroundColor', 'w', ...
                         'Value',u(1), ...
                         'Min', min_val,'Max', max_val,...
                         'callback', {@get_slider_values,gui,u,q0,q,dq,ddq});    
set(gui.slider_u1, 'Units', 'normalized')

% U2 Slider
gui.slider_u2 = uicontrol('style','slider',...
                         'string','2',...
                         'units','normalized',...
                         'position',[slider_left slider_bottom-slider_height slider_width slider_height],...
                         'fontsize',14,...
                         'BackgroundColor', 'w', ...
                         'Value',u(2), ...
                         'Min', min_val,'Max', max_val,...
                         'callback', {@get_slider_values,gui,u,q0,q,dq,ddq});    
set(gui.slider_u2, 'Units', 'normalized')

% U3 Slider
gui.slider_u3 = uicontrol('style','slider',...
                         'string','3',...
                         'units','normalized',...
                         'position',[slider_left slider_bottom-2*slider_height slider_width slider_height],...
                         'fontsize',14,...
                         'BackgroundColor', 'w', ...
                         'Value',u(3), ...
                         'Min', min_val,'Max', max_val,...
                         'callback', {@get_slider_values,gui,u,q0,q,dq,ddq});    
set(gui.slider_u3, 'Units', 'normalized')

% U4 Slider
gui.slider_u4 = uicontrol('style','slider',...
                         'string','4',...
                         'units','normalized',...
                         'position',[slider_left slider_bottom-3*slider_height slider_width slider_height],...
                         'fontsize',14,...
                         'BackgroundColor', 'w', ...
                         'Value',u(4), ...
                         'Min', min_val,'Max', max_val,...
                         'callback', {@get_slider_values,gui,u,q0,q,dq,ddq});    
set(gui.slider_u4, 'Units', 'normalized')

% U5 Slider
gui.slider_u5 = uicontrol('style','slider',...
                         'string','5',...
                         'units','normalized',...
                         'position',[slider_left slider_bottom-4*slider_height slider_width slider_height],...
                         'fontsize',14,...
                         'BackgroundColor', 'w', ...
                         'Value',u(5), ...
                         'Min', min_val,'Max', max_val,...
                         'callback', {@get_slider_values,gui,u,q0,q,dq,ddq});    
set(gui.slider_u5, 'Units', 'normalized')

% U7 Slider
gui.slider_u7 = uicontrol('style','slider',...
                          'string','7',...
                         'units','normalized',...
                         'position',[slider_left slider_bottom-5*slider_height slider_width slider_height],...
                         'fontsize',14,...
                         'BackgroundColor', 'w', ...
                         'Value',u(7), ...
                         'Min', min_val,'Max', max_val,...
                         'callback', {@get_slider_values,gui,u,q0,q,dq,ddq});    
set(gui.slider_u7, 'Units', 'normalized')

%% Add 'Next Step' button
pb_left   = 0.67;
pb_bottom = 0.09;
pb_width  = 0.2;
pb_height = 0.1;

gui.push_button = uicontrol('style','push',...
                 'units','normalized',...
                 'position',[pb_left pb_bottom pb_width pb_height],...
                 'fontsize',14,...
                 'string','Next Step',...
                 'callback',{@next_step,gui,dt,model_params,ctrl_params,SC,ST,SA,SK,H,FK,FA,FT,FC,torso_end,torso_com,stancefoot_line, freefoot_line, stanceThigh_line, stanceShank_line, freeThigh_line, freeShank_line, torso_line, body_line});





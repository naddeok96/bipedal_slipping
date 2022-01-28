
function model_params = get_model_params(height, mass)
%{
Inputs
-----------------------------------
height - Total Height of Body [m]
mass   - Total Weight of Body [kg]
-----------------------------------

Biomechanics and Motor Control of Human Movement, Fourth Edition pg 83 and 98
(pg 96 and 99 in pdf)
%}

% Body segment lengths as a fraction of body height
head_length     = 0.130;
kneck_to_ground = 0.936;
hip_to_ground   = 0.53;
knee_to_ground  = 0.285;
ankle_to_ground = 0.039;
foot_length     = 0.152;

% Proximal to COM as a fraction of body segment length
hat_to_thigh_com  = 0.626;
hip_to_thigh_com  = 0.433;
knee_to_shank_com = 0.433;
ankle_to_foot_com = 0.5;

% Body segment mass as a fraction of body mass
hat_mass_ratio   = 0.678;
thigh_mass_ratio = 0.1;
shank_mass_ratio = 0.0465;
foot_mass_ratio  = 0.0145;

% Body segment center of gyration as a fraction of body segment lengths
hat_to_hip_radius_of_gyration    = 0.798;
hip_to_thigh_radius_of_gyration  = 0.54;
knee_to_shank_radius_of_gyration = 0.528;

hat_c_of_g_to_radius_of_gyration   = 0.496;
thigh_c_of_g_to_radius_of_gyration = 0.323;
shank_c_of_g_to_radius_of_gyration = 0.302;

% Populate model_params
model_params.height    = height;
model_parmas.mass      = mass;
model_params.g         = 9.8;        % Acceleration due to gravity
model_params.mufric    = 0.1;        % Coeffiecent of friction 
model_params.xf        = 0.0158076;  % Perpendicular distance from shank to foot center of curvature
model_params.yf        = 0.0960;     % From gaitFile
model_params.R         = 0.26245;    % Radius of foot
model_params.R_freetoe = 0;

model_params.Lh      = head_length * height;                         % Head
model_params.L_torso = 0.4; %(kneck_to_ground - hip_to_ground) * height;   % Torso
model_params.Lt      = 0.443450; % (hip_to_ground - knee_to_ground) * height;    % Thigh
model_params.Ls      = 0.2531860; %(knee_to_ground - ankle_to_ground) * height;  % Shank
model_params.Lf_x    = foot_length * height;                         % Foot Length
model_params.Lf_y    = ankle_to_ground * height;                     % Foot Height

model_params.stanceheel         = [-0.0149649, -0.1111869];
model_params.freetoe            = [0.2725251,   0        ];
model_params.stanceheel2freetoe = -0.3935904;

model_params.c_torso = 0.2    ; % hat_to_thigh_com * (model_params.L_torso + model_params.Lh); % HAT to Hip COM
model_params.ct      = 0.19186; % hip_to_thigh_com * model_params.Lt;           % Hip to Thigh COM
model_params.cs      = 0.19367; % knee_to_shank_com * model_params.Ls;          % Knee to Shank COM
model_params.cf_x    = 0.13756; % ankle_to_foot_com * (foot_length * height);   % Ankle to COM of Foot
model_params.cf_y    = 0.19186;                 % MISSING

model_params.mb      = 30.494 * 2;     % hat_mass_ratio * mass;   % Mass of Torso (HAT)
model_params.mt      = 7.3;            % thigh_mass_ratio * mass; % Mass of Thigh
model_params.ms      = 3.3945;         % shank_mass_ratio * mass; % Mass of Shank
model_params.mf      = 1.0585;         % foot_mass_ratio * mass;  % Mass of Foot
model_params.J_torso = 0.6099 * 30;    % model_params.mb * ((model_params.L_torso + model_params.Lh)^2) * ((hat_to_hip_radius_of_gyration^2)    + (hat_c_of_g_to_radius_of_gyration^2));
model_params.Jt      = 0.149711 * 0.1; % model_params.mt * (model_params.Lt^2) * ((hip_to_thigh_radius_of_gyration^2)  + (thigh_c_of_g_to_radius_of_gyration^2));
model_params.Js      = 0.204956 * 0.1; % model_params.ms * (model_params.Ls^2) * ((knee_to_shank_radius_of_gyration^2) + (shank_c_of_g_to_radius_of_gyration^2));
model_params.Jf      = 0.1      * 0.1; % Moment of interia for foot
model_params.gamma   = 0;   % angle of ground measured CCW

% % Calculate Foot params
while (model_params.R ^ 2) < ((model_params.Lf_x ^ 2)/4) || model_params.R - ((model_params.R ^ 2) - ((model_params.Lf_x ^ 2)/4))^(1/2) > model_params.Lf_y
    model_params.R = model_params.R + 0.01;
end
sagitta = model_params.R - ((model_params.R ^ 2) - ((model_params.Lf_x ^ 2)/4))^(1/2); % Distance from arc to chord of foot 
th      = model_params.Lf_y - sagitta; % Distance from Chord to Ankle
ah      = (1/5) * model_params.Lf_x;   % Distance from heel to ankle on x axis

model_params.Or_x = ((1/2) * model_params.Lf_x) - ah;                              % X-Coord of Or from ankle
model_params.Or_y = (((model_params.R^2) - ((model_params.Lf_x^2)/4))^(1/2)) - th; % Y-Coord of Or from ankle

model_params.h_x  = -ah; % X-Coord of Heel Point from Ankle
model_params.h_y  = -th; % Y-Coord of Heel Point from Ankle

model_params.t_x  = model_params.Lf_x - ah; % X-Coord of Toe Point from Ankle
model_params.t_y  = -th;                    % Y-Coord of Toe Point from Ankle

model_params.theta = 2 * asin((0.5 * model_params.Lf_x) / model_params.R); % Angle about Or from heal to toe
model_params.phi = tan((model_params.Lf_x - ah) / th); % Angle between foot top and line through ankle perpendicular to the chord

end
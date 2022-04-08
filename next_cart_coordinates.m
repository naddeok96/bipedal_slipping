function [SC, ST, SA, SK, H, FK, FA, FT, FC, torso_end, torso_com] = next_cart_coordinates(q, q0, SC, model_params, ST, SA, SK, H, FK, FA, FT, FC, torso_end, torso_com)
% Finds the position of key points for animation
% Inputs:
%   ~ q: an 6xN matrix of joint angles evenly spaced in time.  The
%     first set of angles is assumed to be at the instant after impact.
%   ~ eventIndEven: an 1x(M+1) vector of indices for the instant before
%     impact
%   ~ model_params: data structure containing geometric values
% Outputs - 2xN vectors giving xy-position:
%   ~ SC: stance contact
%   ~ ST: stance foot center of curvature
%   ~ SA: stance ankle
%   ~ SK: stance knee
%   ~ H:  hip
%   ~ FK: free knee
%   ~ FA: free ankle
%   ~ FT: free foot center of curvature
%   ~ FC: lowest point on free foot
%
% Anne Martin
% September 5, 2011
% Modified August 22, 2012
% Modified October 18, 2012

%% Set up
xf      = model_params.xf;
R       = model_params.R;
Lt      = model_params.Lt;
Ls      = model_params.Ls;
gamma   = model_params.gamma;
L_torso = model_params.L_torso;
c_torso = model_params.c_torso;

%% One step
% Stance Leg
SC        = SC + R * ((q0(1)+q0(3)+q0(5))-(q(1)+q(3)+q(5)))*[cos(gamma); sin(gamma)];
ST        = SC + [-(R*sin(gamma)); R*cos(gamma)];
SA        = ST + [-(xf*sin(gamma + q(1) + q(3) + q(5))); xf*cos(gamma + q(1) + q(3) + q(5))];
SK        = SA + [-(Ls*sin(gamma + q(1) + q(3))); Ls*cos(gamma + q(1) + q(3))];

% Hip
H         = SK + [-(Lt*sin(gamma + q(1))); Lt*cos(gamma + q(1))];

% Free Leg
FK        =  H + [Lt*sin(gamma + q(1) + q(2)); -(Lt*cos(gamma + q(1) + q(2)))];
FA        = FK + [Ls*sin(gamma + q(1) + q(2) + q(4)); -(Ls*cos(gamma + q(1) + q(2) + q(4)))];
FT        = FA + [xf*sin(gamma + q(1) + q(2) + q(4) + q(6)); -xf*cos(gamma + q(1) + q(2) + q(4) + q(6))];
FC        = FT + [R*sin(gamma); -(R*cos(gamma))];

% HAT
torso_end =  H + [L_torso*sin(gamma+q(1)+q(7));-L_torso*cos(gamma+q(1)+q(7))];
torso_com =  H + [c_torso*sin(gamma+q(1)+q(7));-c_torso*cos(gamma+q(1)+q(7))];



function [SC, ST, SA, SK, H, FK, FA, FT, FC, torso_end, torso_com] = get_cart_coordinates(q, model_params)
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

%% Initial position
FC         = zeros(2,1);
FT         = FC - [R * sin(gamma), -(R * cos(gamma))]';
FA         = FT - [xf*sin(gamma + q(1) + q(2) + q(4) + q(6)), -xf*cos(gamma + q(1) + q(2) + q(4) + q(6))]';
FK         = FA - [Ls*sin(gamma + q(1) + q(2) + q(4)),-(Ls*cos(gamma + q(1) + q(2) + q(4)))]';
H          = FK - [Lt*sin(gamma + q(1) + q(2)),-(Lt*cos(gamma + q(1) + q(2)))]';
SK         =  H - [-(Lt*sin(gamma + q(1))),Lt*cos(gamma + q(1))]';
SA         = SK - [-(Ls*sin(gamma + q(1) + q(3))),  Ls*cos(gamma + q(1) + q(3))]';
ST         = SA - [-(xf*sin(gamma + q(1) + q(3) + q(5))), xf*cos(gamma + q(1) + q(3) + q(5))]';
SC         = ST - [-(R*sin(gamma)), R*cos(gamma)]';
torso_end  =  H - [-L_torso*sin(gamma+q(1)+q(7)),L_torso*cos(gamma+q(1)+q(7))]';
torso_com  =  H - [-c_torso*sin(gamma+q(1)+q(7)),c_torso*cos(gamma+q(1)+q(7))]';

end

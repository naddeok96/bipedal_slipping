function [Ge , G] = get_Ge_7links_sgl(q, model_params)
% Calculates the inertia matrix for single support
% Inputs:
%  ~ q: a vector of joint angles
%  ~ model_params: a data structure containing all of the model parameters
% Outputs:
%  ~ D: the inertia matrix
%  ~ G: the gravity matrix
%
% author:  Anne Martin
% created: 10/12/2012
% ported to MATLAB: 10/13/2013

% Set up
% Get the mass and length parameters out of the data structure
Lt      = model_params.Lt;      % Length of thigh
Ls      = model_params.Ls;      % Length of shank
ct      = model_params.ct;      % Distance from COM of thigh to hip
cs      = model_params.cs;      % Distance from COM of shank to thigh
c_torso = model_params.c_torso; % Distance from COM of torso to hip
xf      = model_params.xf;      % Perpendicular distance from shank to foot center of curvature
Mb      = model_params.mb;      % Mass of torso
Mt      = model_params.mt;      % Mass of thigh
Ms      = model_params.ms;      % Mass of shank
g       = model_params.g;       % Accerleration due to gravity

% Get the angles
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q7 = q(7);

% G matrix
Ge = zeros(9,1);
Ge(1) = -g*(Mb*xf*sin(q1 + q3 + q5) - Ms*cs*sin(q1 + q2 + q4) + 2*Ms*xf*sin(q1 + q3 + q5) + 2*Mt*xf*sin(q1 + q3 + q5) + Ls*Mb*sin(q1 + q3) + 2*Ls*Ms*sin(q1 + q3) - Lt*Ms*sin(q1 + q2) + 2*Ls*Mt*sin(q1 + q3) - Mb*c_torso*sin(q1 + q7) - Ms*cs*sin(q1 + q3) - Mt*ct*sin(q1 + q2) + Lt*Mb*sin(q1) + Lt*Ms*sin(q1) + 2*Lt*Mt*sin(q1) - Mt*ct*sin(q1));
Ge(2) = g*(Ms*cs*sin(q1 + q2 + q4) + Lt*Ms*sin(q1 + q2) + Mt*ct*sin(q1 + q2));
Ge(3) = -g*(Mb*xf*sin(q1 + q3 + q5) + 2*Ms*xf*sin(q1 + q3 + q5) + 2*Mt*xf*sin(q1 + q3 + q5) + Ls*Mb*sin(q1 + q3) + 2*Ls*Ms*sin(q1 + q3) + 2*Ls*Mt*sin(q1 + q3) - Ms*cs*sin(q1 + q3));
Ge(4) = Ms*cs*g*sin(q1 + q2 + q4);
Ge(5) = -g*xf*sin(q1 + q3 + q5)*(Mb + 2*Ms + 2*Mt);
Ge(7) = Mb*c_torso*g*sin(q1 + q7);
Ge(9) = g*(Mb + 2*Ms + 2*Mt);

G=Ge(1:7,1);


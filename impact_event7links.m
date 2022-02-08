function [x_plus, f] = impact_event7links(x_minus, model_params)
% Applies the impact map and calculates impulse
% Inputs:
%  ~ x_minus: vector of joint angles and angular velocities at the instant
%    before impact
%  ~ model_params: struct containing model parameters
% Outputs:
% ~ x_plus: vector of joint angles and angular velocities at the instant
%   after impact
% ~ f: vector of horizontal and vertical impulsive forces
%
% author:   Anne Martin
% created:  7/19/2012
% modified: 10/15/2012
% ported to MATLAB: 10/13/2013

%% Set up
% Parse x_minus
q_minus  = x_minus(1:7);
dq_minus = x_minus(8:14);

% Get the impact matrices
[De, E, S] = get_impact_matrices7links(q_minus, model_params);

%% Find q_plus
q_plus = S*q_minus;
x_plus(1:7) = q_plus;

%% Now find dq_plus
% Set up the left hand side
a = [
    De, -E'
    E,  zeros(2,2)];

% % Find the hip velocity at the instant before impact
% [~, E2] = get_impact_matrices7links(q_plus, model_params);
% vH = -E2(:,1:6)*S*dq_minus;
% dqe=[dq_minus; vH];
% In Anne Martin's code this was the velocity of the Hip position, but in my (Kuo Chen)
% definition the extended variable velocity dx,dy is zero if stance foot pure rolling

% Set up the right hand side
dqe = [dq_minus; 0;0];
b = [De*dqe; zeros(2,1)];

% Solve
sol = a\b;
x_plus(8:14) = S*sol(1:7);
f = sol(10:11);

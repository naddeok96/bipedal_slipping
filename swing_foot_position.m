function [h, S] = swing_foot_position(q, qPlus, model_params)
% Calculates the position of the swing foot
% Accepts multiple timesteps
% Inputs:
%  ~ q: matrix of joint angles
%  ~ qPlus: vector of joint angles at the instant after impact.  If the relative position
%    of the feet is important and not the actual distance traveled, then q can be passed in for qPlus.
%  ~ model_params: struct containing model parameters
% Outputs:
%  ~ h: height of swing foot
%  ~ S: horizontal position of swing foot
%
% author:   Anne Martin
% created:  7/15/2011
% modified: 10/11/2012
% ported to MATLAB: 10/13/2013

%% Set up
Lt = model_params.Lt;
Ls = model_params.Ls;
R  = model_params.R;
xf = model_params.xf;

q1Plus = qPlus(1);
q3Plus = qPlus(3);
q5Plus = qPlus(5);

%% Calculate where the lowest point of the foot is
n = size(q,2);
h = zeros(1,n);
S = zeros(1,n);
for i=1:n
    q1 = q(1,i);
    q2 = q(2,i);
    q3 = q(3,i);
    q4 = q(4,i);
    q5 = q(5,i);
    q6 = q(6,i);
    h(1,i) = Lt*cos(q1) - Lt*cos(q1 + q2) + Ls*cos(q1 + q3) - Ls*cos(q1 + q2 + q4) + xf*cos(q1 + q3 + q5) - xf*cos(q1 + q2 + q4 + q6);
    S(1,i) = (-q1 - q3 - q5 + q1Plus + q3Plus + q5Plus)*R - (Lt*sin(q1)) + Lt*sin(q1 + q2) - Ls*sin(q1 + q3) + Ls*sin(q1 + q2 + q4) - xf*sin(q1 + q3 + q5) + xf*sin(q1 + q2 + q4 + q6);
end

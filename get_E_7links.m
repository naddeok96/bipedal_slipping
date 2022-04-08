function [E] = get_E_7links(q,model_params)
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
Lt    = model_params.Lt;
Ls    = model_params.Ls;
ct    = model_params.ct;
cs    = model_params.cs;
cf_x  = model_params.cf_x;
cf_y  = model_params.cf_y;
R     = model_params.R;
xf    = model_params.xf;
Mb    = model_params.mb;
Mt    = model_params.mt;
Ms    = model_params.ms;
mf    = model_params.mf;
Jt    = model_params.Jt;
Js    = model_params.Js;
% foot moment of inertia otherwise De is singular
Jf    = model_params.Jf;

g     = model_params.g;
gamma = model_params.gamma;

% Get the angles
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);

% E matrix
E = zeros(2,9);
E(1,1) = Lt*cos(q1 + q2) - Ls*cos(q1 + q3) + xf*cos(q1 + q2 + q4 + q6) - Lt*cos(q1) + Ls*cos(q1 + q2 + q4) - xf*cos(q1 + q3 + q5);
E(1,2) = R + Lt*cos(q1 + q2) + xf*cos(q1 + q2 + q4 + q6) + Ls*cos(q1 + q2 + q4);
E(1,3) =  -R - Ls*cos(q1 + q3) - xf*cos(q1 + q3 + q5);
E(1,4) = R + xf*cos(q1 + q2 + q4 + q6) + Ls*cos(q1 + q2 + q4);
E(1,5) = -R - xf*cos(q1 + q3 + q5);
E(1,6) = R + xf*cos(q1 + q2 + q4 + q6);
E(1,8) = 1;
E(2,1) = Lt*sin(q1 + q2) - Ls*sin(q1 + q3) + xf*sin(q1 + q2 + q4 + q6) - Lt*sin(q1) + Ls*sin(q1 + q2 + q4) - xf*sin(q1 + q3 + q5);
E(2,2) = Lt*sin(q1 + q2) + xf*sin(q1 + q2 + q4 + q6) + Ls*sin(q1 + q2 + q4);
E(2,3) = - Ls*sin(q1 + q3) - xf*sin(q1 + q3 + q5);
E(2,4) = xf*sin(q1 + q2 + q4 + q6) + Ls*sin(q1 + q2 + q4);
E(2,5) = -xf*sin(q1 + q3 + q5);
E(2,6) = xf*sin(q1 + q2 + q4 + q6);
E(2,9) = 1;


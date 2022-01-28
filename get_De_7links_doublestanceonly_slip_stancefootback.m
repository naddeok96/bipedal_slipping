function [D, D_s] = get_De_7links_doublestanceonly_slip_stancefootback(q, model_params)
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

stanceheel_x=model_params.freetoe(1);
stanceheel_y=model_params.freetoe(2);
R_stanceheel=model_params.R_freetoe;
% this is the difference from the double stance nonslip program. The stance
% foot is behind free foot. So the contact part of stance foot is toe not
% heel

% freetoe_x=model_params.freetoe(1);
% freetoe_y=model_params.freetoe(2);

% R_freetoe=model_params.R_freetoe;

Mb    = model_params.mb;
Mt    = model_params.mt;
Ms    = model_params.ms;
Jt    = model_params.Jt;
Js    = model_params.Js;
% foot moment of inertia otherwise De is singular
Jf    = model_params.Jf;
J_torso=model_params.J_torso;
c_torso=model_params.c_torso;


% Get the angles
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);
% Inertia Matrix during single support
% D matrix
D = zeros(9);
D(1,1) = J_torso + 2*Jf + 2*Js + 2*Jt + Mt*(R_stanceheel + Ls*cos(q1 + q3) + Lt*cos(q1) - ct*cos(q1) + stanceheel_x*cos(q1 + q3 + q5) - stanceheel_y*sin(q1 + q3 + q5))^2 + Ms*(Ls*sin(q1 + q3) - cs*sin(q1 + q3) + stanceheel_y*cos(q1 + q3 + q5) + stanceheel_x*sin(q1 + q3 + q5))^2 + Ms*(R_stanceheel + Ls*cos(q1 + q3) - cs*cos(q1 + q3) + stanceheel_x*cos(q1 + q3 + q5) - stanceheel_y*sin(q1 + q3 + q5))^2 + Ms*(Ls*sin(q1 + q3) - Lt*sin(q1 + q2) + Lt*sin(q1) - cs*sin(q1 + q2 + q4) + stanceheel_y*cos(q1 + q3 + q5) + stanceheel_x*sin(q1 + q3 + q5))^2 + Ms*(R_stanceheel + Ls*cos(q1 + q3) - Lt*cos(q1 + q2) + Lt*cos(q1) - cs*cos(q1 + q2 + q4) + stanceheel_x*cos(q1 + q3 + q5) - stanceheel_y*sin(q1 + q3 + q5))^2 + Mb*(Ls*sin(q1 + q3) - c_torso*sin(q1 + q7) + Lt*sin(q1) + stanceheel_y*cos(q1 + q3 + q5) + stanceheel_x*sin(q1 + q3 + q5))^2 + Mt*(Ls*sin(q1 + q3) - ct*sin(q1 + q2) + Lt*sin(q1) + stanceheel_y*cos(q1 + q3 + q5) + stanceheel_x*sin(q1 + q3 + q5))^2 + Mb*(R_stanceheel + Ls*cos(q1 + q3) - c_torso*cos(q1 + q7) + Lt*cos(q1) + stanceheel_x*cos(q1 + q3 + q5) - stanceheel_y*sin(q1 + q3 + q5))^2 + Mt*(R_stanceheel + Ls*cos(q1 + q3) - ct*cos(q1 + q2) + Lt*cos(q1) + stanceheel_x*cos(q1 + q3 + q5) - stanceheel_y*sin(q1 + q3 + q5))^2 + Mt*(Ls*sin(q1 + q3) + Lt*sin(q1) - ct*sin(q1) + stanceheel_y*cos(q1 + q3 + q5) + stanceheel_x*sin(q1 + q3 + q5))^2;
D(1,2) = Jf + Js + Jt + Lt^2*Ms + Ms*cs^2 + Mt*ct^2 - Lt^2*Ms*cos(q2) - Lt*Ms*cs*cos(q2 + q4) - Mt*R_stanceheel*ct*cos(q1 + q2) + 2*Lt*Ms*cs*cos(q4) - Lt*Mt*ct*cos(q2) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Ls*Lt*Ms*cos(q2 - q3) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) - Ls*Mt*ct*cos(q2 - q3) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ls*Ms*cs*cos(q2 - q3 + q4) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) - Lt*Ms*R_stanceheel*cos(q1 + q2);
D(1,3) = Jf + Js + Ls^2*Mb + 2*Ls^2*Ms + 2*Ls^2*Mt + Mb*R_stanceheel^2 + 2*Ms*R_stanceheel^2 + 2*Mt*R_stanceheel^2 + Ms*cs^2 + Mb*stanceheel_x^2 + Mb*stanceheel_y^2 + 2*Ms*stanceheel_x^2 + 2*Ms*stanceheel_y^2 + 2*Mt*stanceheel_x^2 + 2*Mt*stanceheel_y^2 - 2*Ls*Ms*cs - Mb*R_stanceheel*c_torso*cos(q1 + q7) - 2*Ms*R_stanceheel*cs*cos(q1 + q3) - Mt*R_stanceheel*ct*cos(q1 + q2) + Mb*c_torso*stanceheel_y*sin(q3 + q5 - q7) + Lt*Mb*stanceheel_x*cos(q3 + q5) + Lt*Ms*stanceheel_x*cos(q3 + q5) + 2*Lt*Mt*stanceheel_x*cos(q3 + q5) + Ls*Lt*Mb*cos(q3) + Ls*Lt*Ms*cos(q3) + 2*Ls*Lt*Mt*cos(q3) + Lt*Mb*R_stanceheel*cos(q1) + Lt*Ms*R_stanceheel*cos(q1) + 2*Lt*Mt*R_stanceheel*cos(q1) - Lt*Mb*stanceheel_y*sin(q3 + q5) - Lt*Ms*stanceheel_y*sin(q3 + q5) - 2*Lt*Mt*stanceheel_y*sin(q3 + q5) - Mt*ct*stanceheel_x*cos(q3 + q5) - Ls*Mt*ct*cos(q3) - Mt*R_stanceheel*ct*cos(q1) + Mt*ct*stanceheel_y*sin(q3 + q5) + 2*Ls*Mb*stanceheel_x*cos(q5) + 4*Ls*Ms*stanceheel_x*cos(q5) + 4*Ls*Mt*stanceheel_x*cos(q5) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Ls*Lt*Ms*cos(q2 - q3) - 2*Ls*Mb*stanceheel_y*sin(q5) - 4*Ls*Ms*stanceheel_y*sin(q5) - 4*Ls*Mt*stanceheel_y*sin(q5) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - 2*Ms*cs*stanceheel_x*cos(q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) - Ls*Mb*c_torso*cos(q3 - q7) - Ls*Mt*ct*cos(q2 - q3) + 2*Ms*cs*stanceheel_y*sin(q5) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) + 2*Mb*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Ms*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Mt*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) - 2*Mb*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Ms*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Mt*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ls*Ms*cs*cos(q2 - q3 + q4) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) + 2*Ls*Mb*R_stanceheel*cos(q1 + q3) + 4*Ls*Ms*R_stanceheel*cos(q1 + q3) - Lt*Ms*R_stanceheel*cos(q1 + q2) + 4*Ls*Mt*R_stanceheel*cos(q1 + q3) - Mb*c_torso*stanceheel_x*cos(q3 + q5 - q7);
D(1,4) = Jf + Js + Ms*cs^2 - Lt*Ms*cs*cos(q2 + q4) + Lt*Ms*cs*cos(q4) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ls*Ms*cs*cos(q2 - q3 + q4) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5);
D(1,5) = Jf + Mb*R_stanceheel^2 + 2*Ms*R_stanceheel^2 + 2*Mt*R_stanceheel^2 + Mb*stanceheel_x^2 + Mb*stanceheel_y^2 + 2*Ms*stanceheel_x^2 + 2*Ms*stanceheel_y^2 + 2*Mt*stanceheel_x^2 + 2*Mt*stanceheel_y^2 - Mb*R_stanceheel*c_torso*cos(q1 + q7) - Ms*R_stanceheel*cs*cos(q1 + q3) - Mt*R_stanceheel*ct*cos(q1 + q2) + Mb*c_torso*stanceheel_y*sin(q3 + q5 - q7) + Lt*Mb*stanceheel_x*cos(q3 + q5) + Lt*Ms*stanceheel_x*cos(q3 + q5) + 2*Lt*Mt*stanceheel_x*cos(q3 + q5) + Lt*Mb*R_stanceheel*cos(q1) + Lt*Ms*R_stanceheel*cos(q1) + 2*Lt*Mt*R_stanceheel*cos(q1) - Lt*Mb*stanceheel_y*sin(q3 + q5) - Lt*Ms*stanceheel_y*sin(q3 + q5) - 2*Lt*Mt*stanceheel_y*sin(q3 + q5) - Mt*ct*stanceheel_x*cos(q3 + q5) - Mt*R_stanceheel*ct*cos(q1) + Mt*ct*stanceheel_y*sin(q3 + q5) + Ls*Mb*stanceheel_x*cos(q5) + 2*Ls*Ms*stanceheel_x*cos(q5) + 2*Ls*Mt*stanceheel_x*cos(q5) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Ls*Mb*stanceheel_y*sin(q5) - 2*Ls*Ms*stanceheel_y*sin(q5) - 2*Ls*Mt*stanceheel_y*sin(q5) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - Ms*cs*stanceheel_x*cos(q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) + Ms*cs*stanceheel_y*sin(q5) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) + 2*Mb*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Ms*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Mt*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) - 2*Mb*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Ms*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Mt*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) + Ls*Mb*R_stanceheel*cos(q1 + q3) + 2*Ls*Ms*R_stanceheel*cos(q1 + q3) - Lt*Ms*R_stanceheel*cos(q1 + q2) + 2*Ls*Mt*R_stanceheel*cos(q1 + q3) - Mb*c_torso*stanceheel_x*cos(q3 + q5 - q7);
D(1,6) = Jf;
D(1,7) = J_torso + Mb*c_torso^2 - Mb*R_stanceheel*c_torso*cos(q1 + q7) + Mb*c_torso*stanceheel_y*sin(q3 + q5 - q7) - Lt*Mb*c_torso*cos(q7) - Ls*Mb*c_torso*cos(q3 - q7) - Mb*c_torso*stanceheel_x*cos(q3 + q5 - q7);
D(1,8) = Ms*cs*cos(q1 + q2 + q4) - 2*Ms*R_stanceheel - 2*Mt*R_stanceheel - Mb*R_stanceheel - Mb*stanceheel_x*cos(q1 + q3 + q5) - 2*Ms*stanceheel_x*cos(q1 + q3 + q5) - 2*Mt*stanceheel_x*cos(q1 + q3 + q5) + Mb*stanceheel_y*sin(q1 + q3 + q5) + 2*Ms*stanceheel_y*sin(q1 + q3 + q5) + 2*Mt*stanceheel_y*sin(q1 + q3 + q5) - Ls*Mb*cos(q1 + q3) - 2*Ls*Ms*cos(q1 + q3) + Lt*Ms*cos(q1 + q2) - 2*Ls*Mt*cos(q1 + q3) + Mb*c_torso*cos(q1 + q7) + Ms*cs*cos(q1 + q3) + Mt*ct*cos(q1 + q2) - Lt*Mb*cos(q1) - Lt*Ms*cos(q1) - 2*Lt*Mt*cos(q1) + Mt*ct*cos(q1);
D(1,9) = Ms*cs*sin(q1 + q2 + q4) - Mb*stanceheel_y*cos(q1 + q3 + q5) - 2*Ms*stanceheel_y*cos(q1 + q3 + q5) - 2*Mt*stanceheel_y*cos(q1 + q3 + q5) - Mb*stanceheel_x*sin(q1 + q3 + q5) - 2*Ms*stanceheel_x*sin(q1 + q3 + q5) - 2*Mt*stanceheel_x*sin(q1 + q3 + q5) - Ls*Mb*sin(q1 + q3) - 2*Ls*Ms*sin(q1 + q3) + Lt*Ms*sin(q1 + q2) - 2*Ls*Mt*sin(q1 + q3) + Mb*c_torso*sin(q1 + q7) + Ms*cs*sin(q1 + q3) + Mt*ct*sin(q1 + q2) - Lt*Mb*sin(q1) - Lt*Ms*sin(q1) - 2*Lt*Mt*sin(q1) + Mt*ct*sin(q1);
D(2,1) = Jf + Js + Jt + Lt^2*Ms + Ms*cs^2 + Mt*ct^2 - Lt^2*Ms*cos(q2) - Lt*Ms*cs*cos(q2 + q4) - Mt*R_stanceheel*ct*cos(q1 + q2) + 2*Lt*Ms*cs*cos(q4) - Lt*Mt*ct*cos(q2) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Ls*Lt*Ms*cos(q2 - q3) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) - Ls*Mt*ct*cos(q2 - q3) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ls*Ms*cs*cos(q2 - q3 + q4) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) - Lt*Ms*R_stanceheel*cos(q1 + q2);
D(2,2) = Jf + Js + Jt + Lt^2*Ms + Ms*cs^2 + Mt*ct^2 + 2*Lt*Ms*cs*cos(q4);
D(2,3) = - Mt*R_stanceheel*ct*cos(q1 + q2) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Ls*Lt*Ms*cos(q2 - q3) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) - Ls*Mt*ct*cos(q2 - q3) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ls*Ms*cs*cos(q2 - q3 + q4) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) - Lt*Ms*R_stanceheel*cos(q1 + q2);
D(2,4) = Jf + Js + Ms*cs^2 + Lt*Ms*cs*cos(q4);
D(2,5) = - Mt*R_stanceheel*ct*cos(q1 + q2) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) - Lt*Ms*R_stanceheel*cos(q1 + q2);
D(2,6) = Jf;
D(2,8) = Ms*cs*cos(q1 + q2 + q4) + Lt*Ms*cos(q1 + q2) + Mt*ct*cos(q1 + q2);
D(2,9) = Ms*cs*sin(q1 + q2 + q4) + Lt*Ms*sin(q1 + q2) + Mt*ct*sin(q1 + q2);
D(3,1) = Jf + Js + Ls^2*Mb + 2*Ls^2*Ms + 2*Ls^2*Mt + Mb*R_stanceheel^2 + 2*Ms*R_stanceheel^2 + 2*Mt*R_stanceheel^2 + Ms*cs^2 + Mb*stanceheel_x^2 + Mb*stanceheel_y^2 + 2*Ms*stanceheel_x^2 + 2*Ms*stanceheel_y^2 + 2*Mt*stanceheel_x^2 + 2*Mt*stanceheel_y^2 - 2*Ls*Ms*cs - Mb*R_stanceheel*c_torso*cos(q1 + q7) - 2*Ms*R_stanceheel*cs*cos(q1 + q3) - Mt*R_stanceheel*ct*cos(q1 + q2) + Mb*c_torso*stanceheel_y*sin(q3 + q5 - q7) + Lt*Mb*stanceheel_x*cos(q3 + q5) + Lt*Ms*stanceheel_x*cos(q3 + q5) + 2*Lt*Mt*stanceheel_x*cos(q3 + q5) + Ls*Lt*Mb*cos(q3) + Ls*Lt*Ms*cos(q3) + 2*Ls*Lt*Mt*cos(q3) + Lt*Mb*R_stanceheel*cos(q1) + Lt*Ms*R_stanceheel*cos(q1) + 2*Lt*Mt*R_stanceheel*cos(q1) - Lt*Mb*stanceheel_y*sin(q3 + q5) - Lt*Ms*stanceheel_y*sin(q3 + q5) - 2*Lt*Mt*stanceheel_y*sin(q3 + q5) - Mt*ct*stanceheel_x*cos(q3 + q5) - Ls*Mt*ct*cos(q3) - Mt*R_stanceheel*ct*cos(q1) + Mt*ct*stanceheel_y*sin(q3 + q5) + 2*Ls*Mb*stanceheel_x*cos(q5) + 4*Ls*Ms*stanceheel_x*cos(q5) + 4*Ls*Mt*stanceheel_x*cos(q5) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Ls*Lt*Ms*cos(q2 - q3) - 2*Ls*Mb*stanceheel_y*sin(q5) - 4*Ls*Ms*stanceheel_y*sin(q5) - 4*Ls*Mt*stanceheel_y*sin(q5) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - 2*Ms*cs*stanceheel_x*cos(q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) - Ls*Mb*c_torso*cos(q3 - q7) - Ls*Mt*ct*cos(q2 - q3) + 2*Ms*cs*stanceheel_y*sin(q5) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) + 2*Mb*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Ms*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Mt*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) - 2*Mb*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Ms*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Mt*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ls*Ms*cs*cos(q2 - q3 + q4) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) + 2*Ls*Mb*R_stanceheel*cos(q1 + q3) + 4*Ls*Ms*R_stanceheel*cos(q1 + q3) - Lt*Ms*R_stanceheel*cos(q1 + q2) + 4*Ls*Mt*R_stanceheel*cos(q1 + q3) - Mb*c_torso*stanceheel_x*cos(q3 + q5 - q7);
D(3,2) = - Mt*R_stanceheel*ct*cos(q1 + q2) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Ls*Lt*Ms*cos(q2 - q3) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) - Ls*Mt*ct*cos(q2 - q3) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ls*Ms*cs*cos(q2 - q3 + q4) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) - Lt*Ms*R_stanceheel*cos(q1 + q2);
D(3,3) = Jf + Js + Ls^2*Mb + 2*Ls^2*Ms + 2*Ls^2*Mt + Mb*R_stanceheel^2 + 2*Ms*R_stanceheel^2 + 2*Mt*R_stanceheel^2 + Ms*cs^2 + Mb*stanceheel_x^2 + Mb*stanceheel_y^2 + 2*Ms*stanceheel_x^2 + 2*Ms*stanceheel_y^2 + 2*Mt*stanceheel_x^2 + 2*Mt*stanceheel_y^2 - 2*Ls*Ms*cs - 2*Ms*R_stanceheel*cs*cos(q1 + q3) + 2*Ls*Mb*stanceheel_x*cos(q5) + 4*Ls*Ms*stanceheel_x*cos(q5) + 4*Ls*Mt*stanceheel_x*cos(q5) - 2*Ls*Mb*stanceheel_y*sin(q5) - 4*Ls*Ms*stanceheel_y*sin(q5) - 4*Ls*Mt*stanceheel_y*sin(q5) - 2*Ms*cs*stanceheel_x*cos(q5) + 2*Ms*cs*stanceheel_y*sin(q5) + 2*Mb*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Ms*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Mt*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) - 2*Mb*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Ms*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Mt*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) + 2*Ls*Mb*R_stanceheel*cos(q1 + q3) + 4*Ls*Ms*R_stanceheel*cos(q1 + q3) + 4*Ls*Mt*R_stanceheel*cos(q1 + q3);
D(3,4) = -Ms*cs*(stanceheel_x*cos(q2 - q3 + q4 - q5) + Ls*cos(q2 - q3 + q4) + stanceheel_y*sin(q2 - q3 + q4 - q5) + R_stanceheel*cos(q1 + q2 + q4));
D(3,5) = Jf + Mb*R_stanceheel^2 + 2*Ms*R_stanceheel^2 + 2*Mt*R_stanceheel^2 + Mb*stanceheel_x^2 + Mb*stanceheel_y^2 + 2*Ms*stanceheel_x^2 + 2*Ms*stanceheel_y^2 + 2*Mt*stanceheel_x^2 + 2*Mt*stanceheel_y^2 - Ms*R_stanceheel*cs*cos(q1 + q3) + Ls*Mb*stanceheel_x*cos(q5) + 2*Ls*Ms*stanceheel_x*cos(q5) + 2*Ls*Mt*stanceheel_x*cos(q5) - Ls*Mb*stanceheel_y*sin(q5) - 2*Ls*Ms*stanceheel_y*sin(q5) - 2*Ls*Mt*stanceheel_y*sin(q5) - Ms*cs*stanceheel_x*cos(q5) + Ms*cs*stanceheel_y*sin(q5) + 2*Mb*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Ms*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Mt*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) - 2*Mb*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Ms*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Mt*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) + Ls*Mb*R_stanceheel*cos(q1 + q3) + 2*Ls*Ms*R_stanceheel*cos(q1 + q3) + 2*Ls*Mt*R_stanceheel*cos(q1 + q3);
D(3,7) = -Mb*c_torso*(stanceheel_x*cos(q3 + q5 - q7) + R_stanceheel*cos(q1 + q7) - stanceheel_y*sin(q3 + q5 - q7) + Ls*cos(q3 - q7));
D(3,8) = Mb*stanceheel_y*sin(q1 + q3 + q5) - 2*Ms*R_stanceheel - 2*Mt*R_stanceheel - Mb*stanceheel_x*cos(q1 + q3 + q5) - 2*Ms*stanceheel_x*cos(q1 + q3 + q5) - 2*Mt*stanceheel_x*cos(q1 + q3 + q5) - Mb*R_stanceheel + 2*Ms*stanceheel_y*sin(q1 + q3 + q5) + 2*Mt*stanceheel_y*sin(q1 + q3 + q5) - Ls*Mb*cos(q1 + q3) - 2*Ls*Ms*cos(q1 + q3) - 2*Ls*Mt*cos(q1 + q3) + Ms*cs*cos(q1 + q3);
D(3,9) = Ms*cs*sin(q1 + q3) - 2*Ms*stanceheel_y*cos(q1 + q3 + q5) - 2*Mt*stanceheel_y*cos(q1 + q3 + q5) - Mb*stanceheel_x*sin(q1 + q3 + q5) - 2*Ms*stanceheel_x*sin(q1 + q3 + q5) - 2*Mt*stanceheel_x*sin(q1 + q3 + q5) - Ls*Mb*sin(q1 + q3) - 2*Ls*Ms*sin(q1 + q3) - 2*Ls*Mt*sin(q1 + q3) - Mb*stanceheel_y*cos(q1 + q3 + q5);
D(4,1) = Jf + Js + Ms*cs^2 - Lt*Ms*cs*cos(q2 + q4) + Lt*Ms*cs*cos(q4) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ls*Ms*cs*cos(q2 - q3 + q4) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5);
D(4,2) = Jf + Js + Ms*cs^2 + Lt*Ms*cs*cos(q4);
D(4,3) = -Ms*cs*(stanceheel_x*cos(q2 - q3 + q4 - q5) + Ls*cos(q2 - q3 + q4) + stanceheel_y*sin(q2 - q3 + q4 - q5) + R_stanceheel*cos(q1 + q2 + q4));
D(4,4) = Jf + Js + Ms*cs^2;
D(4,5) = -Ms*cs*(stanceheel_x*cos(q2 - q3 + q4 - q5) + stanceheel_y*sin(q2 - q3 + q4 - q5) + R_stanceheel*cos(q1 + q2 + q4));
D(4,6) = Jf;
D(4,8) = Ms*cs*cos(q1 + q2 + q4);
D(4,9) = Ms*cs*sin(q1 + q2 + q4);
D(5,1) = Jf + Mb*R_stanceheel^2 + 2*Ms*R_stanceheel^2 + 2*Mt*R_stanceheel^2 + Mb*stanceheel_x^2 + Mb*stanceheel_y^2 + 2*Ms*stanceheel_x^2 + 2*Ms*stanceheel_y^2 + 2*Mt*stanceheel_x^2 + 2*Mt*stanceheel_y^2 - Mb*R_stanceheel*c_torso*cos(q1 + q7) - Ms*R_stanceheel*cs*cos(q1 + q3) - Mt*R_stanceheel*ct*cos(q1 + q2) + Mb*c_torso*stanceheel_y*sin(q3 + q5 - q7) + Lt*Mb*stanceheel_x*cos(q3 + q5) + Lt*Ms*stanceheel_x*cos(q3 + q5) + 2*Lt*Mt*stanceheel_x*cos(q3 + q5) + Lt*Mb*R_stanceheel*cos(q1) + Lt*Ms*R_stanceheel*cos(q1) + 2*Lt*Mt*R_stanceheel*cos(q1) - Lt*Mb*stanceheel_y*sin(q3 + q5) - Lt*Ms*stanceheel_y*sin(q3 + q5) - 2*Lt*Mt*stanceheel_y*sin(q3 + q5) - Mt*ct*stanceheel_x*cos(q3 + q5) - Mt*R_stanceheel*ct*cos(q1) + Mt*ct*stanceheel_y*sin(q3 + q5) + Ls*Mb*stanceheel_x*cos(q5) + 2*Ls*Ms*stanceheel_x*cos(q5) + 2*Ls*Mt*stanceheel_x*cos(q5) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Ls*Mb*stanceheel_y*sin(q5) - 2*Ls*Ms*stanceheel_y*sin(q5) - 2*Ls*Mt*stanceheel_y*sin(q5) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - Ms*cs*stanceheel_x*cos(q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) + Ms*cs*stanceheel_y*sin(q5) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) + 2*Mb*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Ms*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Mt*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) - 2*Mb*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Ms*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Mt*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) + Ls*Mb*R_stanceheel*cos(q1 + q3) + 2*Ls*Ms*R_stanceheel*cos(q1 + q3) - Lt*Ms*R_stanceheel*cos(q1 + q2) + 2*Ls*Mt*R_stanceheel*cos(q1 + q3) - Mb*c_torso*stanceheel_x*cos(q3 + q5 - q7);
D(5,2) = - Mt*R_stanceheel*ct*cos(q1 + q2) - Lt*Ms*stanceheel_x*cos(q2 - q3 - q5) - Lt*Ms*stanceheel_y*sin(q2 - q3 - q5) - Mt*ct*stanceheel_x*cos(q2 - q3 - q5) - Mt*ct*stanceheel_y*sin(q2 - q3 - q5) - Ms*R_stanceheel*cs*cos(q1 + q2 + q4) - Ms*cs*stanceheel_x*cos(q2 - q3 + q4 - q5) - Ms*cs*stanceheel_y*sin(q2 - q3 + q4 - q5) - Lt*Ms*R_stanceheel*cos(q1 + q2);
D(5,3) = Jf + Mb*R_stanceheel^2 + 2*Ms*R_stanceheel^2 + 2*Mt*R_stanceheel^2 + Mb*stanceheel_x^2 + Mb*stanceheel_y^2 + 2*Ms*stanceheel_x^2 + 2*Ms*stanceheel_y^2 + 2*Mt*stanceheel_x^2 + 2*Mt*stanceheel_y^2 - Ms*R_stanceheel*cs*cos(q1 + q3) + Ls*Mb*stanceheel_x*cos(q5) + 2*Ls*Ms*stanceheel_x*cos(q5) + 2*Ls*Mt*stanceheel_x*cos(q5) - Ls*Mb*stanceheel_y*sin(q5) - 2*Ls*Ms*stanceheel_y*sin(q5) - 2*Ls*Mt*stanceheel_y*sin(q5) - Ms*cs*stanceheel_x*cos(q5) + Ms*cs*stanceheel_y*sin(q5) + 2*Mb*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Ms*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Mt*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) - 2*Mb*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Ms*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Mt*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) + Ls*Mb*R_stanceheel*cos(q1 + q3) + 2*Ls*Ms*R_stanceheel*cos(q1 + q3) + 2*Ls*Mt*R_stanceheel*cos(q1 + q3);
D(5,4) = -Ms*cs*(stanceheel_x*cos(q2 - q3 + q4 - q5) + stanceheel_y*sin(q2 - q3 + q4 - q5) + R_stanceheel*cos(q1 + q2 + q4));
D(5,5) = Jf + Mb*R_stanceheel^2 + 2*Ms*R_stanceheel^2 + 2*Mt*R_stanceheel^2 + Mb*stanceheel_x^2 + Mb*stanceheel_y^2 + 2*Ms*stanceheel_x^2 + 2*Ms*stanceheel_y^2 + 2*Mt*stanceheel_x^2 + 2*Mt*stanceheel_y^2 + 2*Mb*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Ms*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) + 4*Mt*R_stanceheel*stanceheel_x*cos(q1 + q3 + q5) - 2*Mb*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Ms*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5) - 4*Mt*R_stanceheel*stanceheel_y*sin(q1 + q3 + q5);
D(5,7) = -Mb*c_torso*(stanceheel_x*cos(q3 + q5 - q7) + R_stanceheel*cos(q1 + q7) - stanceheel_y*sin(q3 + q5 - q7));
D(5,8) = -(Mb + 2*Ms + 2*Mt)*(R_stanceheel + stanceheel_x*cos(q1 + q3 + q5) - stanceheel_y*sin(q1 + q3 + q5));
D(5,9) = -(stanceheel_y*cos(q1 + q3 + q5) + stanceheel_x*sin(q1 + q3 + q5))*(Mb + 2*Ms + 2*Mt);
D(6,1) = Jf;
D(6,2) = Jf;
D(6,4) = Jf;
D(6,6) = Jf;
D(7,1) = J_torso + Mb*c_torso^2 - Mb*R_stanceheel*c_torso*cos(q1 + q7) + Mb*c_torso*stanceheel_y*sin(q3 + q5 - q7) - Lt*Mb*c_torso*cos(q7) - Ls*Mb*c_torso*cos(q3 - q7) - Mb*c_torso*stanceheel_x*cos(q3 + q5 - q7);
D(7,3) = -Mb*c_torso*(stanceheel_x*cos(q3 + q5 - q7) + R_stanceheel*cos(q1 + q7) - stanceheel_y*sin(q3 + q5 - q7) + Ls*cos(q3 - q7));
D(7,5) = -Mb*c_torso*(stanceheel_x*cos(q3 + q5 - q7) + R_stanceheel*cos(q1 + q7) - stanceheel_y*sin(q3 + q5 - q7));
D(7,7) = J_torso + Mb*c_torso^2;
D(7,8) = Mb*c_torso*cos(q1 + q7);
D(7,9) = Mb*c_torso*sin(q1 + q7);
D(8,1) = Ms*cs*cos(q1 + q2 + q4) - 2*Ms*R_stanceheel - 2*Mt*R_stanceheel - Mb*R_stanceheel - Mb*stanceheel_x*cos(q1 + q3 + q5) - 2*Ms*stanceheel_x*cos(q1 + q3 + q5) - 2*Mt*stanceheel_x*cos(q1 + q3 + q5) + Mb*stanceheel_y*sin(q1 + q3 + q5) + 2*Ms*stanceheel_y*sin(q1 + q3 + q5) + 2*Mt*stanceheel_y*sin(q1 + q3 + q5) - Ls*Mb*cos(q1 + q3) - 2*Ls*Ms*cos(q1 + q3) + Lt*Ms*cos(q1 + q2) - 2*Ls*Mt*cos(q1 + q3) + Mb*c_torso*cos(q1 + q7) + Ms*cs*cos(q1 + q3) + Mt*ct*cos(q1 + q2) - Lt*Mb*cos(q1) - Lt*Ms*cos(q1) - 2*Lt*Mt*cos(q1) + Mt*ct*cos(q1);
D(8,2) = Ms*cs*cos(q1 + q2 + q4) + Lt*Ms*cos(q1 + q2) + Mt*ct*cos(q1 + q2);
D(8,3) = Mb*stanceheel_y*sin(q1 + q3 + q5) - 2*Ms*R_stanceheel - 2*Mt*R_stanceheel - Mb*stanceheel_x*cos(q1 + q3 + q5) - 2*Ms*stanceheel_x*cos(q1 + q3 + q5) - 2*Mt*stanceheel_x*cos(q1 + q3 + q5) - Mb*R_stanceheel + 2*Ms*stanceheel_y*sin(q1 + q3 + q5) + 2*Mt*stanceheel_y*sin(q1 + q3 + q5) - Ls*Mb*cos(q1 + q3) - 2*Ls*Ms*cos(q1 + q3) - 2*Ls*Mt*cos(q1 + q3) + Ms*cs*cos(q1 + q3);
D(8,4) = Ms*cs*cos(q1 + q2 + q4);
D(8,5) = -(Mb + 2*Ms + 2*Mt)*(R_stanceheel + stanceheel_x*cos(q1 + q3 + q5) - stanceheel_y*sin(q1 + q3 + q5));
D(8,7) = Mb*c_torso*cos(q1 + q7);
D(8,8) = Mb + 2*Ms + 2*Mt;
D(9,1) = Ms*cs*sin(q1 + q2 + q4) - Mb*stanceheel_y*cos(q1 + q3 + q5) - 2*Ms*stanceheel_y*cos(q1 + q3 + q5) - 2*Mt*stanceheel_y*cos(q1 + q3 + q5) - Mb*stanceheel_x*sin(q1 + q3 + q5) - 2*Ms*stanceheel_x*sin(q1 + q3 + q5) - 2*Mt*stanceheel_x*sin(q1 + q3 + q5) - Ls*Mb*sin(q1 + q3) - 2*Ls*Ms*sin(q1 + q3) + Lt*Ms*sin(q1 + q2) - 2*Ls*Mt*sin(q1 + q3) + Mb*c_torso*sin(q1 + q7) + Ms*cs*sin(q1 + q3) + Mt*ct*sin(q1 + q2) - Lt*Mb*sin(q1) - Lt*Ms*sin(q1) - 2*Lt*Mt*sin(q1) + Mt*ct*sin(q1);
D(9,2) = Ms*cs*sin(q1 + q2 + q4) + Lt*Ms*sin(q1 + q2) + Mt*ct*sin(q1 + q2);
D(9,3) = Ms*cs*sin(q1 + q3) - 2*Ms*stanceheel_y*cos(q1 + q3 + q5) - 2*Mt*stanceheel_y*cos(q1 + q3 + q5) - Mb*stanceheel_x*sin(q1 + q3 + q5) - 2*Ms*stanceheel_x*sin(q1 + q3 + q5) - 2*Mt*stanceheel_x*sin(q1 + q3 + q5) - Ls*Mb*sin(q1 + q3) - 2*Ls*Ms*sin(q1 + q3) - 2*Ls*Mt*sin(q1 + q3) - Mb*stanceheel_y*cos(q1 + q3 + q5);
D(9,4) = Ms*cs*sin(q1 + q2 + q4);
D(9,5) = -(stanceheel_y*cos(q1 + q3 + q5) + stanceheel_x*sin(q1 + q3 + q5))*(Mb + 2*Ms + 2*Mt);
D(9,7) = Mb*c_torso*sin(q1 + q7);
D(9,9) = Mb + 2*Ms + 2*Mt;

D_s=D(1:7,1:7);
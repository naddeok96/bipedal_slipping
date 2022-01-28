function [q0,q,dq,ddq] = switch_stance_joints(q0,q,dq,ddq)
% Switches the joint angles when stance flips
% Inputs:
%  ~ q0: a vector of the joint angles at the last step
%  ~ q: a vector of joint angles
%  ~ dq: a vector of joint velocities
%  ~ ddq: a vector of joint accelerations
% Outputs:
%  ~ q0:  "      " in the opposite stance
%  ~ q:   "      " in the opposite stance
%  ~ dq:  "      " in the opposite stance
%  ~ ddq: "      " in the opposite stance
%
% author:  Kyle Naddeo
% created: 1/28/2022

%% Switching matrix
S  = [
    1  1 0 0 0 0 0
    0 -1 0 0 0 0 0
    0  0 0 1 0 0 0
    0  0 1 0 0 0 0
    0  0 0 0 0 1 0
    0  0 0 0 1 0 0
    0 -1 0 0 0 0 1];

%% Switch joints
q0  = S*q0;
q   = S*q;
dq  = S*dq;
ddq = S*ddq;

end
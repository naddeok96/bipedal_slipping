function [De, E, S] = get_impact_matrices7links(q,model_params)
% Calculate the inertia matrix in extended coordinates and the matrix of
% constraint equations and returns the switching matrix.  These matrices
% are needed to solve for the post-impact velocities given the pre-impact 
% velocities. 
%
% Inputs:
%  ~ q: a vector of joint angles
%  ~ model_params: a data structure containing all of the model parameters
% Outputs:
%  ~ De: the inertia matrix in extended coordinates
%  ~ E: the matrix of constraint equations
%  ~ S: the switching matrix
%
% The extended coordinates are qe = [q; xH; yH] where (xH, yH) are the
% position of the hip.  
% The following system of equations must be solved
%  De*dqe(+) - E'*F = De*dqe(-)
%  E*deq(+) +  0*F = 0
% where De is the inertia matrix in extended coordinates, E is the
% Lagrangian constraint, dqe(+) is the time derivative of the extended
% coordinates at the instant after impact, dqe(-) is the time derivative of
% the extended coordinates at the instant before impact and F is the impact
% forces. 
%
% author:  Anne Martin
% created: 10/15/2012
% ported to MATLAB: 10/13/2013



% Create the matrix that switches the angle definitions
S  = [
    1  1 0 0 0 0 0
    0 -1 0 0 0 0 0
    0  0 0 1 0 0 0
    0  0 1 0 0 0 0
    0  0 0 0 0 1 0
    0  0 0 0 1 0 0
    0 -1 0 0 0 0 1];

%% Calculate De

[De,~]=get_De_7links_doublestanceonly_slip_stancefootback(q,model_params);

%% Calculate E

[E]=get_E_7links(q,model_params);

function [SC, ST, SA, SK, FK, FA, FT, FC] = switch_stance_cart_coordinates(SC0, ST0, SA0, SK0, FK0, FA0, FT0, FC0)
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


%% Impact
SC = FC0;
ST = FT0;
SA = FA0;
SK = FK0;
FK = SK0;
FA = SA0;
FT = ST0;
FC = SC0;


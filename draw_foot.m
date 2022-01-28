function [x, y, AA] = draw_foot(r,center,xf,abs_angle,foot_angle,phi_b,phi_t)
    % given a radius, center point and abs shank angle, draws a foot
    % Inputs:
    %  ~ r: the radius of the circle
    %  ~ center: the coordinates of the center point of the circle, pass in as
    %    a vector
    %  ~ abs_angle: the abs angle of the foot at given moment 
    %  ~ foot_angle: the angle of the foot relative to the shank (q5 or q6)

    

    % Plot the foot
    theta = linspace(abs_angle - phi_b - pi, abs_angle + phi_t - pi);
    x = center(1)+r.*cos(theta);
    y = center(2)+r.*sin(theta);
    x(end+1) = x(1);
    y(end+1) = y(1);

    % Determine where the the foot and shank intersect
    xAA = -xf*sin(foot_angle)*(sin(phi_b)+sin(phi_t))/(cos(phi_b-foot_angle)-cos(foot_angle+phi_t)) + r*sin(phi_b+phi_t)*cos(foot_angle)/(cos(phi_b-foot_angle)-cos(foot_angle+phi_t));
    yAA = xAA*(cos(phi_b)-cos(phi_t))/(sin(phi_b)+sin(phi_t)) - r*sin(phi_b+phi_t)/(sin(phi_b)+sin(phi_t));
    AA = [
         xAA*sin(abs_angle) + yAA*cos(abs_angle) + center(1)
        -xAA*cos(abs_angle) + yAA*sin(abs_angle) + center(2)];
end


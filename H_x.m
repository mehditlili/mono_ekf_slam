function out = H_x(x_r, y_r, theta, x_l, y_l, z_l, focal_length)
%H derivative of the measure model with respect to robot and marker pos
%   Detailed explanation goes here

out = [[ -(focal_length*(2*x_l - 2*x_r))/(2*z_l*((x_l - x_r)^2 + (y_l - y_r)^2)^(1/2)), -(focal_length*(2*y_l - 2*y_r))/(2*z_l*((x_l - x_r)^2 + (y_l - y_r)^2)^(1/2)), 0, (focal_length*(2*x_l - 2*x_r))/(2*z_l*((x_l - x_r)^2 + (y_l - y_r)^2)^(1/2)), (focal_length*(2*y_l - 2*y_r))/(2*z_l*((x_l - x_r)^2 + (y_l - y_r)^2)^(1/2)), -(focal_length*((x_l - x_r)^2 + (y_l - y_r)^2)^(1/2))/z_l^2]
[                                  -(y_l - y_r)/((x_l - x_r)^2 + (y_l - y_r)^2),                                   (x_l - x_r)/((x_l - x_r)^2 + (y_l - y_r)^2), 1,                                  (y_l - y_r)/((x_l - x_r)^2 + (y_l - y_r)^2),                                 -(x_l - x_r)/((x_l - x_r)^2 + (y_l - y_r)^2),                                                           0]];
end


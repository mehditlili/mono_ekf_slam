function [out, out_pixel] = h(x_r, y_r, theta, x_l, y_l, z_l, focal_length)
%H measurement model

out = zeros(2, 1);
out_pixel = zeros(2, 1);

out(1) = sqrt((x_l - x_r)*(x_l - x_r) + (y_l - y_r)*(y_l - y_r))*focal_length/z_l;
out(2) = pi/2 - atan2(y_l-y_r, x_l-x_r) + theta;
out_pixel(1) = out(1)*cos(out(2));
out_pixel(2) = out(1)*sin(out(2));
end



%jacobian([sqrt((x_l - x_r)*(x_l - x_r) + (y_l - y_r)*(y_l - y_r))*focal_length/z_l,...
%          pi/2 - atan2(y_l-y_r, x_l-x_r) + theta], [x_r, y_r, theta, x_l, y_l, z_l])


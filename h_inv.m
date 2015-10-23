function out = h_inv(z_r, z_phi, l_z, r_x, r_y, theta, focal_length)
%H_INV reprojects observed landmark from image plan to world
% usins a assumption for landmark's depth
% z_r and z_phi are the pixel observations in polar coordinates
%   Detailed explanation goes here

x_cam = z_r*sin(z_phi)*l_z/focal_length;
y_cam = z_r*cos(z_phi)*l_z/focal_length;

rot_mat = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

out = rot_mat*[x_cam;y_cam; l_z] + [r_x; r_y; 0];

end


% syms z_r z_phi l_z r_x r_y theta focal_length
% x_cam = z_r*sin(z_phi)*l_z/focal_length;
% y_cam = z_r*cos(z_phi)*l_z/focal_length;
% rot_mat = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
% jacobian(rot_mat*[x_cam;y_cam; l_z] + [r_x; r_y; 0], [r_x, r_y, theta])
% jacobian(rot_mat*[x_cam;y_cam; l_z] + [r_x; r_y; 0], [z_r, z_phi])
% jacobian(rot_mat*[x_cam;y_cam; l_z] + [r_x; r_y; 0], [l_z])
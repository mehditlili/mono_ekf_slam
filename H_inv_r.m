function out = H_inv_r(z_r, z_phi, l_z, r_x, r_y, theta, focal_length)
%H_INV_X derivative of back projection with respect to robot state
%   Detailed explanation goes here

out = [[ 1, 0, - (l_z*z_r*cos(theta)*cos(z_phi))/focal_length - (l_z*z_r*sin(theta)*sin(z_phi))/focal_length]
       [ 0, 1,   (l_z*z_r*cos(theta)*sin(z_phi))/focal_length - (l_z*z_r*cos(z_phi)*sin(theta))/focal_length]
       [ 0, 0,                                                                                             0]];
end


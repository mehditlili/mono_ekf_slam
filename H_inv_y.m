function out = H_inv_y(z_r, z_phi, l_z, r_x, r_y, theta, focal_length)
%H_INV_Y Derivativ e of back projection with respect to observation (angle
%and radius)
%   Detailed explanation goes here

out = [[ (l_z*cos(theta)*sin(z_phi))/focal_length - (l_z*cos(z_phi)*sin(theta))/focal_length, (l_z*z_r*cos(theta)*cos(z_phi))/focal_length + (l_z*z_r*sin(theta)*sin(z_phi))/focal_length]
       [ (l_z*cos(theta)*cos(z_phi))/focal_length + (l_z*sin(theta)*sin(z_phi))/focal_length, (l_z*z_r*cos(z_phi)*sin(theta))/focal_length - (l_z*z_r*cos(theta)*sin(z_phi))/focal_length]
       [                                                                                   0,                                                                                           0]];
end


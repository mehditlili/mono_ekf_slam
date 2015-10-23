function out = H_inv_z(z_r, z_phi, l_z, r_x, r_y, theta, focal_length)
%H_INV_Z derivative of back projection with respect to the unobserved
%variable z (landmark depth)
%   Detailed explanation goes here
out = [...
        (z_r*cos(theta)*sin(z_phi))/focal_length - (z_r*cos(z_phi)*sin(theta))/focal_length;
        (z_r*cos(theta)*cos(z_phi))/focal_length + (z_r*sin(theta)*sin(z_phi))/focal_length;
                                                                                   1];
end


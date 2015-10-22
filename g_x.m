function out = g_x(v, omega, theta, dt)
%G_X Summary of this function goes here
%   Detailed explanation goes here
out = zeros(3, 1);
out(1) = -v/omega * sin(theta) + v/omega * sin(theta + omega*dt);
out(2) = v/omega * cos(theta) - v/omega * cos(theta + omega*dt);
out(3) = omega*dt;

end


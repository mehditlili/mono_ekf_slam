function [ out ] = G_x(v, omega, theta, dt)
%G_X Jacobian of motion function, dg/dx
out = zeros(3, 3);
out(1, 3) = v/omega * (-cos(theta) + cos(theta + omega*dt));
out(2, 3) = v/omega * (-sin(theta) + sin(theta + omega*dt));

end


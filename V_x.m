function out = V_x(v, omega, theta, dt)
%V dg/du
out = zeros(3, 2);
out(1, 1) = (-sin(theta)+ sin(theta + omega*dt))/omega;
out(1, 2) = v*(sin(theta) - sin(theta + omega*dt))/(omega*omega) + v*cos(theta+omega*dt)*dt/omega;
out(2, 1) = (cos(theta) - cos(theta + omega*dt)) / omega;
out(2, 2) = -v*(cos(theta) - cos(theta + omega*dt))/(omega*omega) + v*sin(theta+omega*dt)*dt/omega;
out(3, 2) = dt;
end


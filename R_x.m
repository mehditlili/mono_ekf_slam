function out = R_x(v, omega, theta, dt)
%R noise for covariance update
%   Detailed explanation goes here
out = V_x(v, omega, theta, dt)*M_x(v,omega)*V_x(v, omega, theta, dt)';
end


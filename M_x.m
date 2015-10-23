function out = M_x(v, omega)
%M Covariance matrix of noise in control space
alpha1 = 0.001;
alpha2 = 0.001;
alpha3 = 0.001;
alpha4 = 0.001;
out = zeros(2, 2);
out(1, 1) = alpha1*v*v + alpha2*omega*omega;
out(2, 2) = alpha3*v*v + alpha4*omega*omega;

end


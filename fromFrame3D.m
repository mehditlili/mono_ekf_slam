function [p, P_r, P_pr] = fromFrame3D(r, p_r)

t = r(1:3);
a_yaw = r(6);
R_yaw = [cos(a_yaw) -sin(a_yaw) 0; sin(a_yaw) cos(a_yaw) 0 ; 0 0 1];
a_pitch = r(5);
R_pitch = [cos(a_pitch) 0 sin(a_pitch); 0 1 0; -sin(a_pitch) 0 cos(a_pitch)];
a_roll = r(4);
R_roll = [1 0 0; 0 cos(a_roll) -sin(a_roll); 0 sin(a_roll) cos(a_roll)];

R = R_yaw*R_pitch*R_roll;
%R = angle2dcm(a(3), a(2), a(1));
p = R*p_r + t;

if nargout > 1
    px = p_r(1);
    py = p_r(2);
    pz = p_r(3);
    P_r = [...
            [ 1, 0, 0, 0, 0, - py*cos(yaw) - px*sin(yaw)]
            [ 0, 1, 0, 0, 0,   px*cos(yaw) - py*sin(yaw)]
            [ 0, 0, 1, 0, 0,                           0]];
    P_pr = R;    
end

end

%%
function f()
%%
syms x y z roll pitch yaw px py pz real
r = [x y z roll pitch yaw]';
p_r = [px py pz]';
p = fromFrame3D(r, p_r);
p_r = jacobian(p, r)

end
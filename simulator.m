clear all
% generate landmarks array
number_of_markers = 36;
sqr_size = sqrt(number_of_markers);
landmarks = zeros(number_of_markers, 3);
id = 1;
fov = 12;
for x=(fov-sqr_size)/2:(fov-sqr_size)/2 + sqr_size-1
    for y=(fov-sqr_size)/2:(fov-sqr_size)/2 + sqr_size-1
     landmarks(id, :) = [x, y, 3];
     id = id+1;
    end
end


% I. INITIALIZE
%


%   0. System def.
% System Noise
%q = [0.1; 0.1]; %amount of noise movement of robot
%Q = diag(q.^2);

% Sensor noise
sensor_noise = [2; 2*pi/180];
S_noise = diag(sensor_noise.^2);
% Initial z assumption and its uncertainty
init_l_z = 2.8;
P_z = 0.8;

randn('seed', 2);
%   1. Simulator
%       R: robot pose
%       u: control

R = [6; 6;0]; %robot starts at the origin
u = [3; 2]; %Move forward and turn
u_const = u;
W = landmarks';
%   2. Estimator
robotSize = 3;
markerSize = 3;
x = zeros(robotSize+markerSize*size(W, 2), 1);
P = zeros(numel(x), numel(x));
mapspace = 1:numel(x);
l = zeros(2, size(W, 2));
r = find(mapspace, robotSize);
mapspace(r) = 0; %Reserve space for robot state variables
%P(logical(eye(size(P)))) = 0;
x(r) = R;
P(r, r) = 0;


%   3. Graphics
cla
mapFig = figure(1);
axis([0 12 0 12 0 5])
WG = line('parent', gca, ...
    'linestyle', 'none', ...
    'marker', '+', ...
    'color', 'k',...
    'xdata', W(1, :),...
    'ydata', W(2, :),...
    'zdata', W(3, :));

for lid=1:size(W,2)
    text(W(1, lid), W(2, lid), W(3, lid), num2str(lid), 'VerticalAlignment','bottom', ...
                     'HorizontalAlignment','right', 'FontSize',8, 'Color', 'k')

end


init_markers = line('parent', gca, ...
    'linestyle', 'none', ...
    'marker', 'o', ...
    'color', 'm',...
    'xdata', [],...
    'ydata', [],...
    'zdata', []);

RG = line('parent', gca, ...
    'linestyle', 'none', ...
    'marker', '*', ...
    'color', 'g', ...
    'xdata', R(1), ...
    'ydata', R(2), ...
    'zdata', 0);


rG = line('parent', gca, ...
    'linestyle', 'none', ...
    'marker', '*', ...
    'color', 'r', ...
    'xdata', x(r(1)), ...
    'ydata', x(r(2)), ...
    'zdata', 0);

lG = line('parent', gca, ...
    'linestyle', 'none', ...
    'marker', '+', ...
    'color', 'b', ...
    'xdata', [], ...
    'ydata', [], ...
    'zdata', []);

eG = zeros(1, size(W, 2));
for i = 1:size(W, 2)
    eG(i) = line(...
            'parent', gca, ...
            'color', 'g', ...
            'xdata', [],...
            'ydata', [],...
            'zdata', []);
end

    reG= line(...
            'parent', gca, ...
            'color', 'm', ...
            'xdata', [],...
            'ydata', []);
        
polarfig = figure(3);

% polarPoints = polar('parent', gca, ...
%     'linestyle', 'none', ...
%     'marker', '+', ...
%     'theta', [], ...
%     'rho', []);

meshfig = figure(2);

% II. Temporal loop
dt = 0.1;
y = zeros(2, size(W, 2));
y_pixel = zeros(2, size(W, 2));
known_markers = zeros(1, size(W, 2));
focal_length =  200;
for t = 1:1000
    %u = u_const*sin(t/10.0);
    v = u(1);
    omega = u(2);
    theta = R(3);
    % 1. Simulator
    % Robot move with real world noise
    q = diag(M_x(v, omega));
    n = q.*randn(2, 1);
    R = R + g_x(v+n(1), omega+n(2), theta, dt);
    if R(3) > pi
        R(3) = R(3) - 2*pi;
    end
    if R(3) < -pi
        R(3) = R(3) + 2*pi;
    end
    
    
    % Show ground truth in simulator
    set(RG, 'xdata', R(1), 'ydata', R(2), 'zdata', 0);
    
    figure(polarfig);
    cla
    hold on
    % Landmarks are observed with noise
    for lid = 1:size(W,2)
        noise = sensor_noise.*randn(2, 1);
        [y(:, lid), y_pixel(:, lid)] = h(R(1), R(2), R(3),W(1, lid),W(2, lid),W(3, lid), focal_length); 
        y(:, lid) = y(:, lid)  + noise;
        text(y_pixel(1,lid), y_pixel(2, lid), num2str(lid));
    end
    plot(y_pixel(1,:), y_pixel(2, :), '*')
    axis([-500 500 -500 500]);

    %polar(y(2,:), y(1, :), '*')
    hold off

    % 2. Robot observation
    % Kalman prediction step
    v = u(1);
    omega = u(2);
    theta = x(3);
    N = (size(x, 1) - robotSize)/markerSize;
    F_x = zeros(robotSize, robotSize+3*N);
    F_x(1:robotSize, 1:robotSize) = eye(3,3);
    % Predict robot movement from odom input
    x_old = x;
    x = x + F_x'*g_x(v, omega, theta, dt);
    if x(3) > pi
        x(3) = x(3) - 2*pi;
    end
    if x(3) < -pi
        x(3) = x(3) + 2*pi;
    end
    
    % Update the covariance matrix concerning robot state
    G = eye(numel(x)) + F_x'*G_x(v, omega, theta, dt)*F_x;
    P = G*P*G' + F_x'*R_x(v, omega, theta, dt)*F_x;
    % Show predicted robot movement
    set(rG, 'xdata', x(1), 'ydata', x(2), 'zdata', 0);


    
    % Landmark observation and correction 
    % Check if some unknown markers wer observed:
    % For now all markers are always observed, so init all at first
    % iteration
    
    %Landmark initialization
    if t <= size(y,2)
        lid=t;
        %Init mean value of the landmark by back projection the first
        %observsation, here assuming that depth is init_l_z
        x(1+lid*3:(lid+1)*3) = h_inv(y(1, lid), y(2, lid), init_l_z, x(1), x(2), x(3), focal_length);
        %Init the covariance of that landmark in the P matrix
        H_INV_R = H_inv_r(y(1, lid), y(2, lid), init_l_z, x(1), x(2), x(3), focal_length);
        H_INV_Y = H_inv_y(y(1, lid), y(2, lid), init_l_z, x(1), x(2), x(3), focal_length);
        H_INV_Z = H_inv_z(y(1, lid), y(2, lid), init_l_z, x(1), x(2), x(3), focal_length);
        P_lid_lid = H_INV_R*P(1:robotSize , 1:robotSize)*H_INV_R' + ...
                    H_INV_Y*S_noise*H_INV_Y'+...
                    H_INV_Z*P_z*H_INV_Z';

        P_r_x = P(1:robotSize, 1:robotSize+((lid-1)*markerSize));       
        P_lid_x = H_INV_R*P_r_x;
        p_ll = 1+robotSize+(lid-1)*markerSize:3+robotSize+(lid-1)*markerSize;
        p_lx1 = 1+robotSize+(lid-1)*markerSize:3+robotSize+(lid-1)*markerSize;
        p_lx2 = 1:robotSize+(lid-1)*markerSize;
        P(p_ll, p_ll) = P_lid_lid;
        P(p_lx1, p_lx2) = P_lid_x;
        P(p_lx2, p_lx1) = P_lid_x'; 
        known_markers(lid) = 1;
    end
    
    %Landmark observation and state update
    z_prediction = zeros(2, size(y,2));
    z_prediction_pixel = zeros(2, size(y,2));
    figure(meshfig);
    mesh(P); 
    angles = zeros(size(y,2), 1);
    x_predicted = x;
    for lid=1:size(y,2)
        if known_markers(lid)
            angles(lid) = x(3);
            % Marker prediction
            [z_prediction(:, lid), z_prediction_pixel(:, lid)] = h(x(1), x(2), x(3), x(3*lid+1), x(3*lid+2), x(3*lid+3), focal_length);
            F_x_lid = zeros(robotSize+markerSize, robotSize+markerSize*size(W,2));
            F_x_lid(1:3, 1:3) = eye(3);
            F_x_lid(4:6, robotSize+(lid-1)*markerSize +1:robotSize+(lid-1)*markerSize +1 + 2) = eye(3);
            %figure(meshfig);
            %mesh(F_x_lid);
            %drawnow
            H_lid = H_x(x(1), x(2), x(3), x(3*lid+1), x(3*lid+2), x(3*lid+3), focal_length)*F_x_lid;
            K_lid = P*H_lid'*inv(H_lid*P*H_lid' + S_noise);
            x_updated_old = x;
            x = x + K_lid*(y(:,lid) - z_prediction(:, lid));


            dx = x(1:3) - x_updated_old(1:3);
            dx = dx(1:2)' * dx(1:2);
            da = abs(x(3) - x_updated_old(3));
            if dx < 1 && da < 0.1
              P = (eye(size(P)) - K_lid*H_lid)*P;
            else
              x = x_predicted;
            end


            if x(3) > pi
                x(3) = x(3) - 2*pi;
            end
            if x(3) < -pi
                x(3) = x(3) + 2*pi;
            end


    %         figure(polarfig);
    %         hold on
    %         plot(z_prediction_pixel(1,lid), z_prediction_pixel(2, lid), '+')
    %         text(z_prediction_pixel(1,lid), z_prediction_pixel(2, lid), num2str(lid));
    %         drawnow;
    %         hold off
        end
    end
    
    dx = x(1:3) - x_predicted(1:3);
    dx = dx(1:2)' * dx(1:2);
    formatSpec = 'For angle %f robot moved %f\n';
    fprintf(formatSpec, x(3), dx)
    angles
    
    set(init_markers, 'xdata', x(4:3:end), 'ydata', x(5:3:end), 'zdata', x(6:3:end));
    for lid=1:size(W, 2)    
        p_ll = 1+robotSize+(lid-1)*markerSize:3+robotSize+(lid-1)*markerSize;
        le = x(p_ll);
        LE = P(p_ll, p_ll);
        [X, Y, Z] = cov3elli(le, LE, 3, 16);
        set(eG(lid), 'xdata', X, 'ydata', Y, 'zdata', Z); 
    end
    
    set(rG, 'xdata', x(1), 'ydata', x(2), 'zdata', 0);
    [X, Y] = cov2elli(x(1:2), P(1:2, 1:2), 3, 16);
    set(reG, 'xdata', X, 'ydata', Y);    
    figure(4)
    imshow((P - min(min(P))/max(max(P))))
    drawnow

end



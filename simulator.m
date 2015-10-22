% generate landmarks array
landmarks = zeros(49, 3);
id = 1;
for x=3:9
    for y=3:9
     landmarks(id, :) = [x, y, 3];
     id = id+1;
    end
end


% I. INITIALIZE
%


%   0. System def.
% System Noise
q = [0.1; 0.1]; %amount of noise movement of robot
Q = diag(q.^2);
% Measurement noise
m = [1;pi/180];
M = diag(m.^2);

randn('seed', 2);
%   1. Simulator
%       R: robot pose
%       u: control

R = [6;4;0]; %robot starts at the origin
u = [3; 1]; %Move forward and turn
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
x(r) = R;
P(r, r) = 0;
P(robotSize+1:end, robotSize+1:end) = 1000;


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
    'zdata', R(3));


rG = line('parent', gca, ...
    'linestyle', 'none', ...
    'marker', '*', ...
    'color', 'r', ...
    'xdata', x(r(1)), ...
    'ydata', x(r(2)), ...
    'zdata', x(r(3)));

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
        
meshfig = figure(2);
plotfig = figure(3);

% II. Temporal loop
dt = 0.1;
y = zeros(2, size(W, 2));
y_pixel = zeros(2, size(W, 2));
known_markers = zeros(1, size(W, 2));
init_l_z = 3;
focal_length =  200;
for t = 1:1000
    v = u(1);
    omega = u(2);
    theta = R(3);
    % 1. Simulator
    % Robot move with real world noise
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

    % Landmarks are observed with noise
    for lid = 1:size(W,2)
        noise = m.*randn(2, 1);
        [y(:, lid), y_pixel(:, lid)] = h(R(1), R(2), R(3),W(1, lid),W(2, lid),W(3, lid), focal_length); 
        y(:, lid) = y(:, lid)  + noise;
    end
    figure(plotfig);
    %plot(y_pixel(2,:), y_pixel(1, :), '*')
    %axis([-500 500 -500 500]);

    polar(y(2,:), y(1, :), '*')

    drawnow;
    
    % 2. Robot observation
    
    % Kalman prediction step
    v = u(1);
    omega = u(2);
    theta = x(3);
    N = (size(x, 1) - robotSize)/markerSize;
    F_x = zeros(robotSize, robotSize+3*N);
    F_x(1:robotSize, 1:robotSize) = eye(3,3);
    % Predict robot movement from odom input
    x = x + F_x'*g_x(v, omega, theta, dt);
    % Update the covariance matrix concerning robot state
    G = eye(numel(x)) + F_x'*G_x(v, omega, theta, dt)*F_x;
    P = G*P*G' + F_x'*R_x(v, omega, theta, dt)*F_x;
    % Show predicted robot movement
    set(rG, 'xdata', x(1), 'ydata', x(2), 'zdata', 0);

    % Landmark observation and correction 
    % Check if some unknown markers wer observed:
    % For now all markers are always observed, so init all at first
    % iteration
    if t == 1
        for lid=1:size(y,2)
            x(1+lid*3:(lid+1)*3) = h_inv(y(1, lid), y(2, lid), init_l_z, x(1), x(2), x(3), focal_length);   
        end
        set(init_markers, 'xdata', x(4:3:end), 'ydata', x(5:3:end), 'zdata', x(6:3:end));    
        drawnow
    end
    
    for lid=1:size(y,2)
        % Marker prediction
        z_prediction = h(x(1), x(2), x(3), x(3*lid+1), x(3*lid+2), x(3*lid+3), focal_length);
        F_x_lid = zeros(robotSize+markerSize, robotSize+markerSize*size(W,2));
        F_x_lid(1:3, 1:3) = eye(3);
        F_x_lid(4:6, robotSize+(lid-1)*markerSize +1:robotSize+(lid-1)*markerSize +1 + 2) = eye(3);
        %figure(meshfig);
        %mesh(F_x_lid);
        %drawnow
        H_lid = H_x(x(1), x(2), x(3), x(3*lid+1), x(3*lid+2), x(3*lid+3), focal_length)*F_x_lid;
        K_lid = P*H_lid'*inv(H_lid*P*H_lid' + M);
        x = x + K_lid*(y(:,lid) - z_prediction);
        P = (eye(size(P)) - K_lid*H_lid)*P;
%         figure(meshfig);
%         mesh(P);
%         drawnow
    end
    
    
    set(rG, 'xdata', x(1), 'ydata', x(2), 'zdata', 0);
    [X, Y] = cov2elli(x(1:2), P(1:2, 1:2), 3, 16);
    set(reG, 'xdata', X, 'ydata', Y);
    figure(meshfig);
    mesh(P);     
    drawnow

end



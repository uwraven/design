addpath(genpath('Nodes'));

% Create vehicle object
vehicle = Vehicle();

% Set vehicle properties
vehicle.m = 15;
vehicle.cf = 0.0006; % kg / s / N
vehicle.J = diag(ones(3, 1));

% Get A, B matrices from EOM
[A, B] = LinearizedDynamics(vehicle.m, vehicle.J);

% Set LQR controller gains
Q = diag([1 0.7 0.7 20 20 20 1 1 1 1 1 1]);
R = diag([1 1 1 10 10 10]);
K = lqr(A, B, Q, R); % Compute gains from linearized EOM, Q, R
vehicle.setLQRGains(K);

% Configure allocation layer
% Limits by (min, max)
% These lims are for the linear allocation strategy
vehicle.allocator.setActuatorLimits([
	60 180
	-18 18
	-18 18
	-10 10
	-10 10
	-5 5
]);
vehicle.allocator.setActuatorGeometry(0.3, 1.0, 0.1);
vehicle.allocator.clamped = true;

% Initial conditions
vehicle.setState([1 0 0 0 0 0 0 0 0 0 0 0]);
vehicle.setReference([0 0 0 0 0 0 0 0 0 0 0 0]);

dt = 0.001;
ts = 0:dt:30;

X = zeros(length(ts), 13);
U = zeros(length(ts), 12);
M = zeros(length(ts), 1);


% Fixed time step integration
for i = 1:length(ts)
	vehicle.tick(dt);
	% Save vehicle states
	X(i, :) = vehicle.x';
	U(i, 1:6) = vehicle.uGlobalRealized';
	U(i, 7:12) = vehicle.uAllocated';
	M(i) = vehicle.m;
	
% 	if ts(i) < 15
% 		vehicle.setReference([5 0 0 0 0 0 0 0 0 0 0 0]);
% 	else
% 		vehicle.setReference([0 0 0 0 0 0 0 0 0 0 0 0]);
% 	end
		
end


%% Plot results
figure(1); clf;
plot3(X(:,3), X(:,2), X(:,1)); grid on;
xlabel('Z'); ylabel('Y'); zlabel('X');
title('Position');

figure(2); clf;
plot(ts, X(:,1:3)); grid on;
title('Position'); legend('X', 'Y', 'Z');

figure(3); clf;
plot(ts, X(:,7:10)); grid on;
title('Attitude'); legend('q0', 'q1', 'q2', 'q3');

figure(4); clf;
plot(ts, U(:,1:6)); grid on;
title('Requested Inputs'); legend('FX', 'FY', 'FZ', 'MX', 'MY', 'MZ');

figure(5); clf;
plot(ts, U(:,7:12)); grid on;
title('Linear Allocated Inputs'); legend('F_E_x', 'F_E_y', 'F_E_z', 'F_R_R', 'F_R_P', 'F_R_Y');

figure(6); clf;
plot(ts, M); grid on;
title('Mass');

% TS = ts(1 : length(ts) / 20 : end);
points = 40;
XS = X(1 : round(length(ts) / points) : end, :);
US = U(1 : round(length(ts) / points) : end, 7:12);

figure(7); clf;
trace3D(XS, US)

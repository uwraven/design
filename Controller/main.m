clc; clear; 
addpath('./');
format long;

raven = Raven();

% Get A, B matrices from EOM
[A, B] = LinearizedDynamics(raven.m, raven.J);

% Set LQR controller gains
Q = diag([1 1 1 20 20 20 1 1 1 1 1 1]);
R = diag([1 1 1 10 10 10]);
K = lqr(A, B, Q, R); % Compute gains from linearized EOM, Q, R


% Initial conditions
raven.setState([0 0 10 0 0 0 0 0 0 0 0 0]);
raven.trajectoryPlanner.targetCoordinate = [0 0 0]';
raven.controller = LQRController();
raven.controller.setGains(K);

dt = 0.001;
ts = 0:dt:60;

X = zeros(length(ts), 13);
U = zeros(length(ts), 21);
M = zeros(length(ts), 1);

% Fixed time step integration
tic();
for i = 1:length(ts)
	raven.update(dt);
	% Save vehicle states
	X(i, :) = raven.x';
	U(i, 1:6) = raven.uRequestedGlobal';
    U(i, 7:12) = raven.uRealizedGlobal';
	U(i, 13:21) = raven.uAllocated';
	M(i) = raven.m;
end
toc()

%%


%% Plot results
figure(1); clf;
plot3(X(:,1), X(:,2), X(:,3)); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Position');

figure(2); clf;
plot(ts, X(:,1:3)); grid on;
title('Position'); legend('X', 'Y', 'Z');

figure(3); clf;
plot(ts, X(:,7:10)); grid on;
title('Attitude'); legend('q0', 'q1', 'q2', 'q3');

% figure(4); clf;
% plot(ts, U(:,1:6)); grid on;
% title('Requested Inputs'); legend('FX', 'FY', 'FZ', 'MX', 'MY', 'MZ');

% figure(5); clf;
% plot(ts, U(:,13:21)); grid on;
% title('Linear Allocated Inputs'); legend('F_E_x', 'F_E_y', 'F_E_z', 'F_R_R', 'F_R_P', 'F_R_Y');

% figure(6); clf;
% plot(ts, M); grid on;
% title('Mass');

%%

% TS = ts(1 : length(ts) / 20 : end);
% points = 40;
% 
% XS = X(1 : round(length(ts) / points) : end, :);
% US = U(1 : round(length(ts) / points) : end, 7:12);
% 
% figure(7); clf;
% trace3D(XS, US)

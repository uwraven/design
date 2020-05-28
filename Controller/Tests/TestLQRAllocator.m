clc; clear;
addpath(genpath('../'));

%% Horizontal maneuver

raven = Raven();
raven.controller.enabled = true;
raven.allocator.enabled = true;

raven.controller = LQRController();
raven.controller.setGains(defaultLQR(raven.m, raven.J));

target = [0 0 0]';
initialState = [1 0 0 0 0 0 0 0 0 0 0 0];
raven.setState(initialState);
raven.trajectoryPlanner.targetCoordinate = target;
raven.trajectoryPlanner.filterCutoffFrequency = 0.5;
raven.rcs.continuous = true;

dt = 0.001;
f = 2;
ts = 0:dt:f;

X = zeros(length(ts), 13);
XR = zeros(length(ts), 3);
U = zeros(length(ts), 6);
UA = zeros(length(ts), 9);
UG = zeros(length(ts), 6);

for i = 1:length(ts)
    raven.update(dt);
    X(i,:) = raven.x';
    XR(i,:) = raven.trajectoryPlanner.filteredCoordinate';
    U(i,:) = raven.uRequestedGlobal';
    UA(i,:) = raven.uAllocated';
    UG(i,:) = raven.uRealizedGlobal';
end

figure(1); clf; hold on;
plot(ts, X(:, 1));
plot(ts, XR(:,1));
ylabel('Position')
legend('x', 'x_r');
title('Position')

figure(2); clf; hold on;
plot(ts, U(:, 1));
plot(ts, UG(:, 1));
legend('f_x', 'f_x_g');
title('requested and realized inputs')

figure(3); clf; hold on;
plot(ts, UA(:, 1:9));
legend('fe_x', 'fe_y', 'fe_z', 'fr_1', 'fr_2', 'fr_3', 'fr_4', 'fr_5', 'fr_6');
title('allocated')

figure(4); clf; hold on;
plot(ts, X(:,7:10));
legend('q_0', 'q_1', 'q_2', 'q_3');
title('Attitude')

deviation = target' - X(end,1:3)
distribution = norm(deviation)

assert(distribution < 1e-3);

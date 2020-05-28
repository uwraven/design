clc; clear;
addpath(genpath('../'));

raven = Raven();

raven.controller = LQRController();
raven.controller.setGains(defaultLQR(raven.m, raven.J));

raven.controller.enabled = true;
raven.allocator.enabled = false;

target = [0 0 0]';
initialState = [1 0 0 0 0 0 0 0 0 0 0 0];
raven.setState(initialState);
raven.trajectoryPlanner.targetCoordinate = target;
raven.trajectoryPlanner.filterCutoffFrequency = 0.5;

dt = 0.001;
f = 10;
ts = 0:dt:f;

X = zeros(length(ts), 13);
XR = zeros(length(ts), 3);
U = zeros(length(ts), 6);

for i = 1:length(ts)
    raven.update(dt);
    X(i,:) = raven.x';
    XR(i,:) = raven.trajectoryPlanner.filteredCoordinate';
    U(i,:) = raven.uRealizedGlobal';
end

figure(1); clf; hold on
plot(ts, X(:, 1));
plot(ts, XR(:,1));
ylabel('Position')
legend('x', 'x_r');

figure(2); clf;
plot(ts, U(:, 1));
legend('f_x');

deviation = target' - X(end,1:3);

assert(norm(deviation) < 1e-3);

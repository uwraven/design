clc; clear;
addpath(genpath('../'));
format long;

%% Horizontal maneuver

raven = Raven();

raven.controller = LQRController();
raven.controller.setGains(defaultLQR());

raven.controller.enabled = true;
raven.allocator.enabled = true;
raven.allocator.clamped = false;

target = [-0.5 0 -5]';
initialState = [0 0 0 0 0 0 0 0.1 0.01 0 0 0];
raven.setState(initialState);
raven.trajectoryPlanner.targetCoordinate = target;
raven.trajectoryPlanner.filterCutoffFrequency = 0.5;
raven.rcs.continuous = true;

dt = 0.001;
f = 30;
ts = 0:dt:f;

X = zeros(length(ts), 13);
XR = zeros(length(ts), 3);
U = zeros(length(ts), 6);
ULR = zeros(length(ts), 6);
UA = zeros(length(ts), 6);
UG = zeros(length(ts), 6);
ULG = zeros(length(ts), 6);

for i = 1:length(ts)
    raven.update(dt);
    X(i,:) = raven.x';
    XR(i,:) = raven.trajectoryPlanner.filteredCoordinate';
    U(i,:) = [
        raven.m * raven.uRequestedGlobal(1:3)
        raven.J * raven.uRequestedGlobal(4:6)
    ]';
    ULR(i,:) = raven.uRequestedLocal';
    UA(i,:) = raven.uAllocated';
    UG(i,:) = raven.uRealizedGlobal';
    ULG(i,:) = raven.uRealizedLocal';
end

plotPosition(1, ts, X(:,1:3), XR(:,1:3));
plotAttitude(2, ts, X(:,7:10));
plotGlobalInputs([3, 4], ts, U, UG)
plotLocalInputs([5, 6], ts, ULR , ULG, raven.allocator.limits)

figure(3); clf; hold on; grid on;
plot(ts, UA(:, 1:6));
legend('fe_x', 'fe_y', 'fe_z', 'fr_1', 'fr_2', 'fr_3', 'fr_4', 'fr_5', 'fr_6');
title('allocated')

figure(7); clf; hold on; grid on;
plot3(X(:,1), X(:,2), X(:,3));
plot3(XR(:,1), XR(:,2), XR(:,3));
plot3(X(1,1), X(1,2), X(1,3), 'xr');
xlabel('x');
ylabel('y');
zlabel('z');
title('3D Trajectory');
legend('True', 'Reference', 'Start');

deviation = target' - X(end,1:3)
distribution = norm(deviation)

assert(distribution < 1e-3);


clc; clear;
addpath(genpath('../'));
format long;

figure(1); clf; grid on;
figure(2); clf; grid on;

targets = [
    0 0 0 0
    2 0 0 -8
    20 0 0 0
];

% SMALL MANEUVER CLAMPED

raven = Raven();
raven.m = 15;
raven.controller.enabled = true;
raven.controller.antiwindupEnabled = true;
raven.allocator.enabled = true;
raven.plantEnabled = true;
raven.rcs.continuous = false;
raven.trajectoryPlanner.setTargets(targets);

raven.allocator.mode = raven.allocator.modes.clamped;
initialState = [0 0 0 0 0 0 0.0 0 0.0 0 0 0];
raven.setState(initialState);

dt = 0.01;
f = 45;
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
    XR(i,:) = [ 
        raven.controller.trajectoryPrefilter.x.output
        raven.controller.trajectoryPrefilter.y.output
        raven.controller.trajectoryPrefilter.z.output
    ];
    U(i,:) = [
        raven.m * raven.uRequestedGlobal(1:3)
        raven.J * raven.uRequestedGlobal(4:6)
    ]';
    ULR(i,:) = raven.uRequestedLocal';
    UA(i,:) = raven.uAllocated';
    UG(i,:) = raven.uRealizedGlobal';
    ULG(i,:) = raven.uRealizedLocal';
end

figure(1); hold on;
plot(ts, X(:,3))

figure(2); hold on;
plot(ts, UG(:,3), 'LineWidth', 2.5, 'Color', [0.6 0.6 0.6])


% SMALL MANEUVER PROJECTED

raven = Raven();
raven.m = 15;
raven.controller.enabled = true;
raven.controller.antiwindupEnabled = false;
raven.allocator.enabled = true;
raven.plantEnabled = true;
raven.rcs.continuous = true;
raven.trajectoryPlanner.setTargets(targets);

raven.allocator.mode = raven.allocator.modes.projected;
initialState = [0 0 0 0 0 0 0.0 0 0.0 0 0 0];
raven.setState(initialState);

dt = 0.01;
f = 45;
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
    XR(i,:) = [ 
        raven.controller.trajectoryPrefilter.x.output
        raven.controller.trajectoryPrefilter.y.output
        raven.controller.trajectoryPrefilter.z.output
    ];
    U(i,:) = [
        raven.m * raven.uRequestedGlobal(1:3)
        raven.J * raven.uRequestedGlobal(4:6)
    ]';
    ULR(i,:) = raven.uRequestedLocal';
    UA(i,:) = raven.uAllocated';
    UG(i,:) = raven.uRealizedGlobal';
    ULG(i,:) = raven.uRealizedLocal';
end

figure(1); hold on;
plot(ts, X(:,3), '-k')
plot(ts, XR(:,3), '--k', 'LineWidth', 1.0, 'Color', [0.3 0.3 0.3]);
legend('Clamped', 'Projected', 'Prefilter Reference')
xlabel('t (s)');
ylabel('z (m)');
title('Clamped vs Projected Allocation')
ylim([-9 2]);

figure(2); hold on;
plot(ts, U(:,3), '--k')
plot(ts, UG(:,3), '-k')
legend('Clamped u_r', 'Clamped u_a', 'Projected u_r', 'Projected u_a');
xlabel('t (s)');
ylabel('u_g_z (m)');
title('Clamped vs Projected Inputs')
ylim([-180 -120]);


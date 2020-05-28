clc; clear;
addpath(genpath('../'));

% Create raven instance
raven = Raven();
raven.controller.enabled = false;
raven.allocator.enabled = false;

% Set initial vehicle state
target = [0 0 0]';
initialState = [0 0 0 0 0 0 0 0 0 0 0 0];
raven.setState(initialState);
raven.trajectoryPlanner.targetCoordinate = target;

dt = 0.001;
f = 20;
ts = 0:dt:f;

X = zeros(length(ts), 13);

for i = 1:length(ts)
    raven.update(dt);
    X(i,:) = raven.x';
end

figure(1); clf;
plot(ts, X(:, 1:3));

deviation = initialState(3) - X(end,3);

assert(norm(deviation) < 1e-3);

clc; clear;
addpath(genpath('../'));
format long;

raven = Raven();
raven.m = 10;
raven.controller.enabled = true;
raven.allocator.enabled = false;
raven.plantEnabled = false;
raven.rcs.continuous = true;

wnx = 1.5;
drx = 1.1;
wnz = 1.0;
drz = 1.0;

kpx = 2;
kdx = 9;
kix = 0;

kpz = 8;
kdz = 6;
kiz = 1.1;

pq = 0.5;
pw = 0.1;

raven.controller.trajectoryController.x.setGains(kpx, kix, kdx);
raven.controller.trajectoryController.y.setGains(kpx, kix, kdx);
raven.controller.trajectoryController.z.setGains(kpz, kiz, kdz);
raven.controller.trajectoryPrefilter.x.setGains(wnx, drx, kpx, kix, kdx);
raven.controller.trajectoryPrefilter.y.setGains(wnx, drx, kpx, kix, kdx);
raven.controller.trajectoryPrefilter.z.setGains(wnz, drz, kpz, kiz, kdz);
raven.controller.rateController.setGains(pq, pw);
raven.trajectoryPlanner.setTargets([
    0 0 0 0
    5 0 0 -1
])
initialState = [0 0 0 0 0 0 0 0 0 0 0 0];
raven.setState(initialState);

dt = 0.001;
f = 60;
ts = 0:dt:f;

X = zeros(length(ts), 13);
XR = zeros(length(ts), 3);
U = zeros(length(ts), 6);
UA = zeros(length(ts), 6);
UG = zeros(length(ts), 6);
FP = zeros(length(ts), 3);

for i = 1:length(ts)
    raven.update(dt);
    % state = raven.x
    X(i,:) = raven.x';
    U(i,:) = [
        raven.m * raven.uRequestedGlobal(1:3)
        raven.J * raven.uRequestedGlobal(4:6)
    ]';
    FP(i,:) = [
        raven.controller.trajectoryPrefilter.x.output
        raven.controller.trajectoryPrefilter.y.output
        raven.controller.trajectoryPrefilter.z.output
    ]';
    % UA(i,:) = raven.uAllocated';
    UG(i,:) = raven.uRealizedGlobal';
end

plotPosition(1, ts, X(:,1:3), FP(:,1:3));
% plotAttitude(2, ts, X(:,7:10));
plotGlobalInputs(3, ts, U, UG);

deviation = raven.trajectoryPlanner.targetCoordinate' - X(end,1:3)

assert(norm(deviation) < 1e-3);

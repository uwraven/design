clc; clear;
addpath(genpath('../'));
format long;

raven = Raven();
raven.m = 15;
raven.controller.enabled = true;
raven.controller.antiwindupEnabled = true;
raven.allocator.enabled = true;
raven.allocator.mode = raven.allocator.modes.projected;
raven.plantEnabled = true;
raven.rcs.continuous = false;
raven.accelerationLimit = norm([0.4 0.4]);
raven.controller.horizontalAccelerationLimit = 0.4;
raven.controller.verticalAccelerationLimit = 2;

wnx = 0.9;
drx = 1.1;
wnz = 0.7;
drz = 1.2;

kpx = 16;
kdx = 6;
kix = 3;
Ktx = 0;

kpz = 12;
kdz = 7;
kiz = 3;
Ktz = 0;

pq = 18;
pw = 24;

raven.controller.trajectoryController.x.setGains(kpx, kix, kdx, Ktx);
raven.controller.trajectoryController.y.setGains(kpx, kix, kdx, Ktx);
raven.controller.trajectoryController.z.setGains(kpz, kiz, kdz, Ktz);
raven.controller.trajectoryPrefilter.x.setGains(wnx, drx, kpx, kix, kdx);
raven.controller.trajectoryPrefilter.y.setGains(wnx, drx, kpx, kix, kdx);
raven.controller.trajectoryPrefilter.z.setGains(wnz, drz, kpz, kiz, kdz);
raven.controller.rateController.setGains(pq, pw);
raven.trajectoryPlanner.setTargets([
    0 0 0 0
    2 0 0 -1
    15 0 0 0
]);
initialState = [0 0 0 0 0 0 0.0 0 0 0 0 0];
raven.setState(initialState);

dt = 0.01;
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

plotPosition(1, ts, X(:,1:3), XR(:,1:3));
plotAttitude(2, ts, X(:,7:10));
plotGlobalInputs(3, ts, U, UG)
plotLocalInputs([5, 6], ts, ULR , ULG, raven.allocator.limits)


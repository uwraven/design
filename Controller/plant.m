clc; clear;
addpath(genpath('Nodes'));

raven = Raven();

raven.setState([0 0 1 0 0 0 0 0 0 0 0 0]);
raven.controller.enabled = false;

dt = 0.001;
ts = 0:dt:30;

X = zeros(length(ts), 13);      % Record vehicle state
U = zeros(length(ts), 15);      % Record vehicle inputs
M = zeros(length(ts), 1);       % Record vehicle mass

for i = 1:length(ts)
    raven.update(dt);
    % Save vehicle states
	X(i, :) = raven.x';
	U(i, 1:6) = raven.uRealizedGlobal';
	U(i, 7:15) = raven.uAllocated';
	M(i) = raven.m;
end
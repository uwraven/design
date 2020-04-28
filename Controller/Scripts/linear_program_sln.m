%% 3D Allocator Linear Programming Solution
% ARCC Design document
% Matt Vredevoogd 02/28/2020

clc; clear;

% Actuator moment arms
l1 = 1; % engine
l2 = 1; % rcs cg
l3 = 0.1; % rcs radial

% Ur = [
% 	0.1
% 	0.005
% 	100
% 	0.02
% 	0.004
% 	0.3
% ];

% H = [ 
% 	1 0 0 -1 -1 0 1 1 0
% 	0 1 0 0 0 -1 0 0 1
% 	0 0 1 0 0 0 0 0 0
% 	0 -l1 0 0 0 -l2 0 0 l2
% 	l1 0 0 l2 l2 0 -l2 -l2 0
% 	0 0 0 -l3 l3 0 -l3 l3 0
% ];

Ur = [
	10
	0
	0
	0
	0
];

H = [ 
	1 0 -1 -1 0 1 1 0
	0 1 0 0 -1 0 0 1
	0 -l1 0 0 -l2 0 0 l2
	l1 0 l2 l2 0 -l2 -l2 0
	0 0 -l3 l3 0 -l3 l3 0
];

% X Y torque allocation 
Aeq = [ Ur, H ];

Beq = Ur;

f = [1 0 0 0 0 0 0 0 0];

lb = [
	0
	-15
	-15
	0
	0
	0
	0
	0
	0
];

ub = [
	1
	15
	15
	10
	10
	10
	10
	10
	10
];

Ua = linprog(f, [], [], Aeq, Beq, lb, ub);

Uu = pinv(H)*Ur
Ua
H*Ua(2:end)


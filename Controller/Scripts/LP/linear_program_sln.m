%% 3D Allocator Linear Programming Solution
% ARCC Design document
% Matt Vredevoogd 02/28/2020

syms Fxr Fyr Fzr Mxr Myr Mzr 'real'; % Requested inputs
syms Fxa Fya Fza Mxa Mya Mza 'real'; % Allocated inputs
syms Fex Fey Fez Frr Frp Fry 'real'; % Actuator commands
syms le lr1 lr2 'real' 'positive'; % Geometry

% Linear input mapping (H)
% (These equations are just for reference)
Fex = Fxr;
Fey = 1 / (le - lr1) * (Mzr - Fyr * lr1);
Fez = 1 / (le - lr1) * (Myr - Fzr * lr1);
Frr = Mxr / (2 * lr2);
Frp = 1 / (le - lr1) * (Fzr * le - Myr);
Fry = 1 / (le - lr1) * (Fyr * le - Mzr);

H = [
	1 0 0 0 0 0
	0 1 0 0 0 1
	0 0 1 0 1 0
	0 0 0 2 * lr2 0 0
	0 0 le 0 lr1 0
	0 le 0 0 0 lr1
];

% Vector of requested inputs
ur = [Fxr Fyr Fzr Mxr Myr Mzr]';

% Vector of actuators
v = [Fex Fey Fez Frr Frp Fry]';

% Compute allocated inputs from the linear mapping
ua = H*v;

% Define the objective function as the difference between allocated and requested
f = ua - ur;

% Constraints
syms Fex_min Fex_max Fey_min Fey_max Fez_min Fez_max Frr_min Frr_max Fry_min Fry_max Frp_min Frp_max 'real';
lb = [Fex_min Fey_min Fez_min Frr_min Frp_min Fry_min]';
ub = [Fex_max Fey_max Fez_max Frr_max Frp_max Fry_max]';

%% Solve the linear program



% lb = subs(lb, lb, [



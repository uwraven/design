load 'engine_params'

% first, solve for nozzle entrance mach number
% vc = mdot / rhoc / (At * ec);
% Mc = vc / mach_speed(k, R, Tc);


% estimate of prandtl number
pr = @(k) 4 * k / (9 * k - 5);

% estimate of dynamic viscosity
mu = @(M, T) 46.6e-10 * sqrt(M) * T ^ 0.6;


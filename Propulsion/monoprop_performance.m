clear; clc;

addpath('isentropic')

% Helpers
psi2pa = @(psi) psi * 6894.76;
r2a = @(r) pi .* r .^ 2;
a2e = @(a) sqrt(a) ./ pi;

% REQUIREMENTS
TW = 1.1;
thrust = 30 * 9.81 * TW; % N

% CHAMBER PARAMETERS
p_c = psi2pa(200); % PSI chamber pressure
t_c = 1000; % 1000K (estimated) decomposition temp

% FLUID PROPERTIES
k = 1.285; % taken from exhaust components of 90% H202
R = 8314.5 / 22.105; % gas constant of decomp products
% assuming complete decomposition of H2O2, and full
% vaporization of H2O
% data from chem tech lib:
% http://www.h2o2.com/technical-library/physical-chemical-properties/thermodynamic-properties/default.aspx?pid=40&name=Decomposition-Products

% AMBIENT PROPERTIES
p_a = 101325;

% STATIC COMPUTED PROPERTIES
e = matched_a_ratio(p_c, p_a, k); % optimal area ratio
v_e = sqrt(2 * k / (k - 1) * R * t_c * (1 - (p_a / p_c) ^ ((k - 1) / k))); % resultant exit velocity

% Mass flow as a function of throat area
m_dot_func = @(A) mdot(A, p_c, k, R, t_c);

% Thrust from mass flow, exit properties
thrust_func = @(m_dot) m_dot * v_e;

% Throat areas
areas = 0:0.001:500;
areas = areas * 1e-6;
md = zeros(1, length(areas));
thr = zeros(1, length(areas));


% Who cares about newtons method! Our computers are faaaaast
for i = 1:length(areas)
	md(i) = m_dot_func(areas(i));
    thr(i) = thrust_func(md(i));
end

inds = abs(thr - thrust) < 0.001;
throat_area = areas(inds);
mass_flow = md(inds);
throat_radius = sqrt(throat_area) / pi;
throat_radius_mm = throat_radius * 1000;










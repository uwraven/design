% Design parameters
thrust = 500;


% Propellant properties %

% Nitrous oxide (ox)
T_o = 293.15; % temperature K
rho_o = 772.25; % density KG / M CU

% Ethanol (f)
T_f = 293.15;
rho_f = 789.0; % KG / M CU
mu_f = 1.2e6; % dynamic viscosity Pa-s
kap_f = 0.167; % thermal conductivity W / m-K
cp_f = 118; % specific heat capacity J / mol K


% CEA DATA %

% Chamber
Tc = 2495.48;
Pc = 2.5000e6;
rhoc = 2.5396-1;
kc = 1.2192;
cp_c = 2.3299; % specific heat kJ / kg K
Mn_c = 21.077; % molar mass

% Throat
Tt = 2242.86;
Pt = 1.3912e6;
rhot = 1.5767e-1;
kt = 1.2397;

% Exit
Te = 1302.46;
Pe = 0.1012e6;
rhoe = 1.9785e-2;
ke = 1.2710;


% Engine Geometry
e = 3.8552; % area ratio
ec = 10.0; % contraction ratio

% Efficiency
cstar = 1516.1;
cf = 1.4260;
isp_vac = 2161.9;

[mdot, At] = mdot_cea(thrust, cf, cstar, Pc);
rt = sqrt(At / pi);

save 'engine_params'

clear
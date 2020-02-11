% Oxidizer Mass and Volume Calculations
% Gabriel Thompson
% 27 December 2019

R_air = 287.058;
R_O2 = 259.8;
T = 300; 

% ro = P/(R*T);
% m = ro*V;
% m = P*V/(R*T);
% I need to have a mass, and pick two pressures
% Mass = 1.2972 kg
% P1 = 120 psi - same as propane
% P2 = 500 psi - mas storage pressure

% m2 - m1 = (P2 - P1)V/RT
% m2*RT/(P2-P1) = V
mp = 1.2972; % Propellant Mass
P1 = 120*6894.75729; % Working (min) pressure, pa
P2 = 500*6894.75729; % Storage (max) pressure, pa
R = R_O2; % Select specific gas constant

V = mp*R*T/(P2 - P1);
disp("Volume: " + V*100^3 + " cubic centimeters");
disp("That's a cube with " + (V*100^3)^(1/3) + " cm sides");
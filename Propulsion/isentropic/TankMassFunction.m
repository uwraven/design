% Tank Mass Calculations
% Gabriel Thompson
% 21 December 2019
clear all;
close all;
% clc;

%% Material Properties setup
% Some common values:
% Ethanol: ro = 789, p = 1.01e3     Propane: ro = 493, p = 0.853e6
% Nitrous: ro = 1220, p = 5.05e6    Helium: ro = 0.179
% Aluminum: ro = 2700, ys = 240e6   Stainless: ro = 8000, ys = 215e6
% Titanium: ro = 4506, ys = 880e6

O_ro = 1450; % Oxidizer density, kg/m3
O_p = 400*6894.76; % Oxidizer pressure, Pa
F_ro = 789; % Fuel density, kg/3
F_p = 101.3e3; % Fuel pressure, Pa
B_ro = 0.179; % Backpressure Gas density, kg/m3
B_p = 20.7e6; % Backpressure Gas pressure, Pa

T_ro = 2700; % Tank density, kg/m3
T_ys = 240e6; % Tank yield strength, Pa
FoS = 3; % Factor of safety, ul
T_ys = T_ys/FoS;

%% Fuel, Oxidizer Mass and Aspect Ratio inputs
% Change these values, user:
    mf = 8; % Dry mass, kg
    t = 45; % Hover time
    ISP = 140; % Specific Impulse
%     m_rat = 1.5; % Mass ratio (m0/mf), ul
    of_rat = 0; % Oxidizer to fuel ratio (O/F), ul
    AR = 1; % Length to Diameter ratio (L/D), L = AR*D = 2*AR*r
    m_rat = exp(t/ISP);
    
m0 = mf*m_rat; % Initial mass, kg    
m_p = m0 - mf; % Propellant mass (m0-mf), kg
F_m = m_p/(of_rat+1); % Fuel mass, kg
O_m = m_p/(of_rat+1)*of_rat; % Oxidizer mass, kg

%% Tank size calculations
% O_v = O_m/O_ro; % Oxidizer volume, m3
O_v = 8.8*1e-3; % 9.6 liter tank
% Volume of a cylinder: V = pi*r^2*L = 2*AR*pi*r^3
% O_Tr = (O_v/(2*AR*pi))^(1/3); % Oxidizer Tank radius, m
% O_TL = 2*AR*O_Tr; % Oxidizer Tank length, m
O_Tr = (6-2*0.0625)/2*0.0254; % 4" diameter
O_sv = 4/3*pi*O_Tr^3; % Sphere volume given a radius (bulkheads)
O_TL = (O_v-O_sv)/(pi*O_Tr^2);
% Hoop stress for a thin walled cylindrical pressure vessel
% sig_z = (P*r)/(2*t)
O_Tt = O_p*O_Tr/(2*T_ys); % Oxidizer Tank thickness, m
% O_Tt = 3.125e-3; % Manually setting thickness to 3.125mm = 1/8"
O_Tv = pi*O_TL*O_Tt*(2*O_Tr+O_Tt) + ... % Cyl. vol.     Oxidizer Tank material volume, m3
    4/3*pi*((O_Tr+O_Tt)^3-O_Tr^3); % Sph. vol. 
O_Tm = O_Tv*T_ro; % Oxidizer Tank mass, kg

% F_v = F_m/F_ro;
% F_Tr = (F_v/(2*AR*pi))^(1/3);
% F_TL = 2*AR*F_Tr;
% % F_Tt = F_p*F_Tr/(2*T_ys);
% F_Tt = 3.125e-3; % Manually setting thickness to 3.125mm = 1/8"
% F_Tv = pi*F_TL*F_Tt*(2*F_Tr+F_Tt)+2*pi*F_Tr^2*F_Tt;
% F_Tm = F_Tv*T_ro;

%% Display Calculations
% disp("For dry mass of " + m0 + " kg and mass ratio of " + m_rat + "...");
% disp("Propellant: " + m_p + " kg: " + F_m + " kg of fuel, " + O_m + " kg of oxidizer.");
disp("O_Tr: " + O_Tr*100/2.54 + " in, O_TL: " + O_TL*100/2.54 + " in.");
disp("O_Tt: " + O_Tt/.0254 + " in, O_Tm: " + O_Tm + " kg.");
% disp("O_v: " + O_v*(100^3) + " cm3"); %, or a cube with " + (O_v*100^3)^(1/3) + " cm sides");
% disp("Oxidizer tank mass: " + O_Tm + " kg.");
% disp("F_Tr: " + F_Tr*100 + " cm, F_TL: " + F_TL*100 + " cm.");
% disp("F_Tt: " + F_Tt*1000 + " mm, F_Tm: " + F_Tm + " kg.");
% disp("F_v: " + F_v*(100^3) + " cm3"); %, or a cube with " + (F_v*100^3)^(1/3) + " cm sides");
% disp("Fuel tank mass: " + F_Tm + " kg.");
% disp("Total tank mass: " + (F_Tm+O_Tm) + " kg.");
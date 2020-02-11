% Rocket Engine Preliminary Design Calculations
% Gabriel Thompson
% 24 December 2019
clear all;
close all;
% clc;

%% Material Properties Setup
% Nitrous Oxide + Propane Stoichiometry
% 10*N2O + C3H8 = 3*CO2 + 4*H2O + 10*N2
global chem_ex_rat;
chem_ex_rat = [3 4 10]; % [CO2 H2O N2] stoich. coefficients of the products
importSpHeatData();
R = getR();

T_0 = 3000; % Combustion temperature, K
P_0 = 0.8*120*6894.75729; % Chamber pressure, pa (1 psi = 6894 Pa)
P_atm = 101325; % Atmospheric pressure, pa
g = getg(3000);

%% Pressure Method
% T/T0 = P/P0^(g-1/g)
T_e = T_0*(P_atm/P_0)^((g-1)/g);
% P/P0 = (1+g-1/2*M2)^(-g/g-1)
M_e = sqrt(2/(g-1)*((P_atm/P_0)^((g-1)/(-g))-1));
A_rat = ((g+1)/2)^(-(g+1)/(2*(g-1)))*(((1+(g-1)/2*M_e^2))/M_e)^((g+1)/(2*(g-1)));
a_e = sqrt(g*R*T_e);
v_e = M_e*a_e;

disp("Mach number: " + M_e + ", Speed of sound: " + a_e + " m/s");
disp("Velocity: " + v_e + " m/s, Exit Tempearture: " + T_e + " K");
disp("Area ratio: " + A_rat);

%% Thrust
% F = mdot*v
m0 = 6.1; % Wet mass, kg
g = 9.81; % Accel. due to gravity, m/s2
mdot = m0*g/v_e;

%% Functions
function g = getg(T)
global sh_CO2_Data sh_H2O_Data sh_N2_Data chem_ex_rat;
gg = [sh_CO2_Data(:,4) sh_H2O_Data(:,4) sh_N2_Data(:,4)]*chem_ex_rat'./sum(chem_ex_rat);
gg = [sh_H2O_Data(:,1) gg];
g = interp1(gg(:,1),gg(:,2),T);
end

function Cp = getCp(T)
global sh_CO2_Data sh_H2O_Data sh_N2_Data chem_ex_rat;
CpCp = chem_ex_rat.*[sh_CO2_Data(:,2) sh_H2O_Data(:,2) sh_N2_Data(:,2)]./sum(chem_ex_rat);
CpCp = [sh_H2O_Data(:,1) CpCp];
Cp = interp1(CpCp(:,1),CpCp(:,2),T);
end

function R = getR()
global chem_ex_rat;
R = chem_ex_rat*[188.92 461.5 296.80]'./sum(chem_ex_rat);
end

function importSpHeatData()
% Each data set contains:       Temp  Cp  Cv  Gamma
global sh_CO2_Data sh_H2O_Data sh_N2_Data;
% CARBON DIOXIDE CO2 CO2 CO2
fid = fopen("basic_CO2SpecificHeat.txt");
for i=1:2
    fgetl(fid);
end
sh_CO2_Data = fscanf(fid,'%g %g', [2 inf])';
R = 188.92;
Cp = sh_CO2_Data(:,2).*1000;
Cv = sh_CO2_Data(:,2).*1000-R;
G = Cp./Cv;
sh_CO2_Data = [sh_CO2_Data(:,1) Cp Cv G];
fclose(fid);

% WATER VAPOR H2O H2O H2O H2O
fid = fopen("basic_H2OSpecificHeat.txt");
for i=1:2
    fgetl(fid);
end
sh_H2O_Data = fscanf(fid,'%g %g', [2 inf])';
R = 461.5;
Cp = sh_H2O_Data(:,2).*1000;
Cv = sh_H2O_Data(:,2).*1000-R;
G = Cp./Cv;
sh_H2O_Data = [sh_H2O_Data(:,1) Cp Cv G];
fclose(fid);

% NITROGEN N2 N2 N2 N2 N2 N2
fid = fopen("basic_N2SpecificHeat.txt");
for i=1:2
    fgetl(fid);
end
sh_N2_Data = fscanf(fid,'%g %g', [2 inf])';
R = 296.80;
Cp = sh_N2_Data(:,2).*1000;
Cv = sh_N2_Data(:,2).*1000-R;
G = Cp./Cv;
sh_N2_Data = [sh_N2_Data(:,1) Cp Cv G];
fclose(fid);
end
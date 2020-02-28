% Gabriel Thompson
% 6 February 2020
% Nitrous Tank Iterative Simulator
clear all; close all; 
% clc;

%% Inputs
% Nominal values: m_dry = 10, MR = 1.65
m_dry = 10;             % Vehicle dry mass, kg
MR = 1.5;               % Vehicle mass ratio
m_lqd = m_dry*(MR-1);   % Fuel mass, kg
F_rcs = 15;              % rcs force, N
t = 45;                 % Simulation run time, s
TWr = 1;                % Thrust-to-weight ratio, ul
Throttle_on = true;    % Enable engine throttling
Heater_on = false;      % Enable heater
Heating_rate = 5;       % Heating rate, J/s

%% Tank Integrator Setup
P_a = 14.7*6894.76; % Ambient pressure, (psi)*(Pa/psi)
T_a = 298; % Ambient Temperature
R_N2O = 189.0; % Individual gas consT_ant for N2O, J/kg*K
T_eng = 1500; % engine Total temperature

T = T_a; % Tank temperature
g = getg(T); % Specific heat ratio
Pvp = vp(T); % Vapor pressure, Pa
P = Pvp; % Tank pressure, kPa

V_Tank = 1.01*m_lqd/745.299; % Tank volume, m3
V_lqd = m_lqd/getrosl(T); % Liquid (fuel) volume, m3
V_vpr = V_Tank - V_lqd; % Ullage (gas) volume, m3
m_vpr = P*V_vpr/(R_N2O*T); % Gas mass, kg

if V_lqd > V_Tank
    disp("Error! Increase Tank volume");
    return
end

Me = sqrt(2/(g-1)*((P_a/P)^((g-1)/(-g))-1)); % Nominal exit mach number
Ar = 1/(Me)*((2+(g-1)*Me^2)/(g+1))^((g+1)/(2*(g-1))); % Area ratio
[Ve_eng,Te_eng,Pe_eng] = getExit(Me,T_eng-(T_a-T),P*0.8);
[Ve_rcs,Te_rcs,Pe_rcs] = getExit(Me,T,P);
F_eng = (m_dry + m_lqd + m_vpr)*9.81*TWr; % Required engine force, N
mdot_vpr = F_rcs/Ve_rcs; % RCS mass flow, kg/s
mdot_lqd = F_eng/Ve_eng; % Engine mass flow, kg/s
m_eva = 0; % Vaporized fuel

H = m_lqd*gethsl(T)*T + m_vpr*gethsv(T)*T; % Total Tank enthalpy, kJ
dH = 0; % Change in enthalpy
dT = 0; % Change in temperature
Q = 0; % Total heat added
Qdot = Heating_rate; % Heating rate, J/s

dt = 0.1; % Time step
tt = 0:dt:t;
thermdata = zeros(5,length(tt));    % T H P Pvp Q
massdata = zeros(5,length(tt));     % m_lqd mdot_lqd m_vpr mdot_vpr m_eva
nozzledata = zeros(6,length(tt));   % Pe_eng Te_eng Ve_eng Pe_rcs Te_rcs Ve_rcs

%% Tank Integrator
for i = 1:1:2 %length(tt)
    % Step 1: Remove mass, expand volume 
    m_lqd = subplus(m_lqd - mdot_lqd*dt);
    m_vpr = subplus(m_vpr - mdot_vpr*dt);
    V_lqd = m_lqd/getrosl(T);
    V_vpr = V_Tank - V_lqd;
    P = m_vpr*R_N2O*T/V_vpr;
    Pvp = vp(T);
    H = m_lqd*getcpsl(T)*T + m_vpr*getcpsv(T)*T;
    
    % Step 2: Vaporize mass to increase pressure
    m_eva = m_vpr*(Pvp/P - 1);
    if m_lqd == 0 || m_eva < 0
        m_eva = 0;
    end
    m_lqd = m_lqd - m_eva;
    m_vpr = m_vpr + m_eva;
    dH = -m_eva*getlhv(T)*T;

    % Step 3: Turn on heater
    if Heater_on && T < T_a
        dQ = Qdot*dt;
        Q = Q + Qdot;
    else
        dQ = 0;
    end
    
    % Step 4: Update temperature, enthalpy, and pressure
    dT = dH/H + dQ/(m_lqd*getcpsl(T) + m_vpr*getcpsv(T));
    T = T + dT;
    H = m_lqd*getcpsl(T)*T + m_vpr*getcpsv(T)*T;
    P = m_vpr*R_N2O*T/V_vpr;
    
    % Step 5: Calculate impact on engine performance
    g = getg(T);
    [Ve_eng,Te_eng,Pe_eng] = getExit(Me,T_eng-(T_a-T),P*0.8);
    [Ve_rcs,Te_rcs,Pe_rcs] = getExit(Me,T,P);
    
    % Step 6: Throttle engine for reduced vehicle weight
    if Throttle_on
       mdot_lqd = (m_dry+m_lqd+m_vpr)*9.81*TWr/Ve_eng;
    end
    
    % Step 7: Store data
    %                 1 2 3 4   5
  	thermdata(:,i) = [T H P Pvp Q];
    %                1     2        3     4        5
    massdata(:,i) = [m_lqd mdot_lqd m_vpr mdot_vpr m_eva];
    %                  1      2      3      4      5      6
    nozzledata(:,i) = [Pe_eng Te_eng Ve_eng Pe_rcs Te_rcs Ve_rcs];
end

%% data Reduction
empt = find(massdata(1,:) == 0);
if isempty(empt)
    empt(1) = 0;
end
disp("Total flight time: " + empt(1)*dt + " s");
disp("Throttle on:" + Throttle_on);
disp("Heater on:" + Heater_on);

% % Generate ideal mass and mass flow curves
% m_idl = (m_dry*MR)./exp(tt./nozzledata(3,1)-m_dry);
% m_idl_dot = -1*[NaN diff(m_idl)./diff(tt)];

%% Plotting
figure();
subplot(2,2,1);
yyaxis left;
plot(tt,massdata(1,:),'r-',tt,massdata(3,:),'b-'); grid on;
title("Mass and Flow");
ylabel("Mass (kg)");
yyaxis right;
plot(tt,massdata(2,:)*1e3,'r--',tt,massdata(4,:)*1e3,'b--'); grid on;
ylabel("Mass Flow (g/s)");
xlabel("Time (s)");
legend("Lqd","Vpr");
ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';

subplot(2,2,2);
yyaxis left;
plot(tt,thermdata(2,:),'r'); grid on;
title("Enthalpy, Heat added");
ylabel("Enthalpy (J)");
yyaxis right;
plot(tt,thermdata(5,:),'b'); grid on;
ylabel("Heat added (J)");
xlabel("Time (s)");
legend("Enthalpy","Heat add.");
ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';

subplot(2,2,3);
title("Pressure");
yyaxis right;
plot(tt,thermdata(1,:),'r'); grid on; hold on;
title("Temperature, Pressure");
ylabel("Tank Temperature (K)");
ylim([200 325]);
yyaxis left;
plot(tt,thermdata(3,:)./6894.76,'b',tt,thermdata(4,:)./6894.76,'k--'); grid on;
ylabel("Tank Pressure (psi)");
ylim([0 1000]);
xlabel("Time (s)");
legend({'Tank P.','Vapor P.','Temp'},"location","southwest");
ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';

subplot(2,2,4);
plot(tt,nozzledata(3,:)./nozzledata(3,1),'r',...
    tt,nozzledata(6,:)./nozzledata(6,1),'b'); grid on;
title("Engine, RCS performance");
ylabel("Thrust variation");
ylim([0.7 1.2]);
xlabel("Time (s)");
legend({'Engine','RCS'},"location","southwest");

%% Vapor pressure vs Temperature
% TT = 250:1:325;
% % pp = vp(TT)./6894.76;
% gg = getg(TT);
% g = getg(298);
% figure(); plot(TT,gg)

%% blaugh
% getrosl(298)

%% Functions
function p = vp(T) % Returns vapor pressure for a given temperature
Pc = 7251e3; % kPa
Tc = 309.57;
Tr = T./Tc;
b = [-6.71839 1.35966 -1.3779 -4.051];
p = Pc.*exp(1./Tr.*(b(1)*(1-Tr) + b(2)*(1-Tr).^(3/2) + ...
    b(3)*(1-Tr).^(5/2) + b(4)*(1-Tr).^5));
end

function h = gethsl(T) % Returns sat. liquid specific enthalpy for given T
Tc = 309.57;
b = [-200 116.043 -917.225 794.779 -589.587];
h = b(1) + b(2).*(1-T./Tc).^(1/3) + b(3).*(1-T./Tc)^(2/3) + ...
    b(4).*(1-T./Tc) + b(5).*(1-T./Tc).^(4/3);
end

function h = gethsv(T) % Returns sat. vapor specific enthalpy for given T
Tc = 309.57;
b = [-200 440.055 -459.701 434.081 -485.338];
h = b(1) + b(2).*(1-T./Tc).^(1/3) + b(3).*(1-T./Tc)^(2/3) + ...
    b(4).*(1-T./Tc) + b(5).*(1-T./Tc).^(4/3);
end

function h = getlhv(T) % Returns latent heat of vaporization for given T
h = gethsv(T)- gethsl(T);
end

function cp = getcp(T)
Tc = 309.57;
Tr = T./Tc;
b = [-0.169903 0.099053 1.20822 -0.248324];
cp = b(1) + b(2)*(Tr).^(-1/2) + b(3)*(Tr).^(1/2) + b(4)*(Tr);
end

function cp = getcpsl(T)
Tc = 309.57;
Tr = T./Tc;
b = [2.49973 0.023454 -3.80136 13.0945 -14.5180];
cp = b(1)*(1 + b(2)*(1-Tr).^(-1) + b(3)*(1-Tr) + b(4)*(1-Tr).^2 + ...
    b(5)*(1-Tr).^3);
end

function cp = getcpsv(T)
Tc = 309.57;
Tr = T./Tc;
b = [ 132.632 0.052187 -0.364923 -1.20233 0.536141];
cp = b(1)*(1 + b(2)*(1-Tr).^(-2/3) + b(3)*(1-Tr).^(-1/3) + ... 
    b(4)*(1-Tr).^(1/3) + b(5)*(1-Tr).^(2/3));
end

function g = getg(T)
R = .189;
g = getcp(T)./(getcp(T)-R);
end

function ro = getrosl(T)
Tc = 309.57;
roc = 452;
b = [1.72328 -0.83950 0.51060 -0.10412];
ro = roc*exp(b(1).*(1-T./Tc).^(1/3) + b(2).*(1-T./Tc)^(2/3) + ...
    b(3).*(1-T./Tc) + b(4).*(1-T./Tc).^(4/3));
% c = [99.3974 0.310729 513.18 0.305143];
end

function ro = getrosv(T)
Tc = 309.57;
roc = 452;
b = [-1.00900 -6.28792 7.50332 -7.90463 0.629427];
ro = roc*exp(b(1).*(Tc./T-1).^(1/3) + b(2).*(Tc./T-1)^(2/3) + ...
    b(3).*(Tc./T-1) + b(4).*(Tc./T-1).^(4/3) + b(5).*(Tc./T-1)^(5/3));
end

function [Ve,Te,Pe] = getExit(Me,T,P)
g = getg(T);
R = 189;
G = (1+(g-1)/2*Me^2);
Pe = P/(G^(-g/(g-1)));
Te = T/G;
a = sqrt(g*R*Te);
Ve = a*Me;
end

function M = getM(Ar,g)
% https://www.grc.nasa.gov/WWW/winddocs/utilities/b4wind_guide/mach.html
% P = 2/(g+1);
% Q = 1-P; % 1 - 2/(g+1)
% R = Ar^(2*Q/P); % Ar^(2*(1/P-1)) = Ar^(g-1)
% a = Q^(1/P); % (1-P)^(1/P) = (1-2/(g+1))^((g+1)/2)
% r = (R-1)/(2*a); % (Ar^(g-1)-1)/(2*(1-2/(g+1))^((g+1)/2))
% X = 1/((1+r)+sqrt(r*(r+2)));
% M = 1/sqrt(X);
r = (Ar^(g-1)-1)/(2*(1-2/(g+1))^((g+1)/2));
M = sqrt((1+r)+sqrt(r*(r+2)));
% More old stuff:
% Vercsnom = sqrt((2*g)/(g-1)*R_N2O*T*(1-(Pa/P)^((g-1)/g))); % Nominal rcs exit velocity
% Veengnom = sqrt((2*g)/(g-1)*R_N2O*T_eng*(1-(Pa/(P*0.8))^((g-1)/g))); % Nominal exit velocity
end
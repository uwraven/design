% Rocket Hover Integrator
% Gabriel Thompson
% 27 December 2019
clear all;
close all;
% clc;

%% User Inputs
ISP = 180;
m_rat = 2;

%% Integrator Setup
mf = 3; % This is arbitrary
m0 = mf*m_rat; 
mp = m0 - mf;
g0 = 9.81;
v = ISP*g0;
t = 0;
m = m0;
tstep = 0.01;
tspan = 1:tstep:100;
mp = [mp NaN(1,length(tspan)-1)];
i = 2;

%% Integrator
while mp(i-1) > 0.08*mp(1) && i < length(tspan)
    t = t + tstep;
    % m*a = mdot*v
    mdot = m*g0/v;
    mp(i) = mp(i-1) - mdot*tstep;
    m = mp(i) + m0;
    i = i + 1;
end

%% Data Reduction
plot(tspan,mp,'r');
axis([0 inf 0 inf]);
xlabel("Time");
ylabel("Propellant Mass");
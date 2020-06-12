clc; clear;
addpath(genpath('../'))

% syms s;         % Signal
s = tf('s');
syms kp kd ki;  % Control gains
syms wn dr;      % Natural frequency, damping

C = kp + ki / s + kd * s;
P = 1 / s^2;
L = C * P;
T = wn^2 / (s^2 + 2 * dr * wn * s + wn^2);
F = (1 + L) * T / L

% LT = L

%% Tuning Translation

w_n = 0.9;
d_r = 1.1;

% k_p = w_n^2;
k_p = 16;
% k_d = 2 * d_r * sqrt(k_p);
k_d = 6;
k_i = 3;

symVars = [kp ki kd wn dr];
numVars = [k_p k_i k_d w_n d_r];

Ls = subs(L, symVars, numVars)
pretty(simplify(F))
Lf = polysym2tf(Ls)

figure(3); clf; hold on; grid on;
margin(Lf);

figure(2); clf; hold on; grid on;
nichols(Lf)

figure(1); clf; hold on; grid on;
Closed_Logarithmic_Nyquist(Lf, 2)


Fs = subs(F, symVars, numVars)


% % fs = ilaplace(Fs)
% % figure(5)
% % fplot(fs, [-1 100])
% % xlabel('ref')

% Ff = polysym2tf(Fs);
% ss(Fs)
% % [num den] = numden(Fs);
% % [A B C D] = tf2ss(num, den)


% %% Tuning Attitude

% pqk = 11.0;
% pwk = 18.0;

% w_n = sqrt(pqk)
% d_r = pqk / 2 / sqrt(pqk)

% C = kp + kd * s;
% P = 1 / s^2;
% L = C * P;

% symVars = [kp ki kd wn d];
% numVars = [pqk 0 pwk w_n d_r];

% Ls = subs(L, symVars, numVars);
% Lf = polysym2tf(Ls);

% figure(4); clf; hold on; grid on;
% margin(Lf)

% figure(2); clf; hold on; grid on;
% nichols(Lf)

% figure(1); clf; hold on; grid on;
% Closed_Logarithmic_Nyquist(Lf, 2)


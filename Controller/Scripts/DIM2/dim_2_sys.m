clear; clc;

% global xi wn J A;

xi = 1.0;
wn = 0.1;
J = 2.9;
A = 1 / J;
alpha = 1.0;

kd = (2 * xi * wn + alpha * wn);
kp = (wn ^ 2 + 2 * xi * wn ^ 2 * alpha);
ki = (alpha * wn ^ 2);

kp = 5.0;
ki = 0.59;
kd = 89.0;


K = 1 / A * [kp ki kd]

%% Open Loop TF analysis

NUM = A * [kd kp ki];
DEN = [1, 0, 0, 0];
sys = tf(NUM, DEN)
z = roots(NUM');
p = roots(DEN');

figure(1); clf; nyquist(sys); grid on;
figure(2); clf; margin(sys); grid on;
figure(3); clf; nichols(sys); grid on;
[Gm, Pm, Wcg, Wcp] = margin(sys);
delay = (Pm) / rad2deg(Wcp)

%% Sensitivity analysis

NUM = [1 0 0 0];
DEN = [1 A * kd A * kp A * ki];
sys = tf(NUM, DEN);
figure(6); clf; margin(sys); grid on;


%% Closed Loop analysis
NUM = A * [kd kp ki];
DEN = [1 A * kd A * kp A * ki];
sys = tf(NUM, DEN);
figure(4); clf; step(sys); grid on;
figure(5); clf; impulse(sys); grid on;
% figure(6); clf; ramp(sys); grid on;



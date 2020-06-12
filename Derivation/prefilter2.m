
s = tf('s');

%%

% Z SYSTEM
wn = 0.4;
dr = 1.1;

kp = 12;
kd = 7;
ki = 3;

C = kp + ki / s + kd * s;
P = 1 / s^2;
L = C * P;
T = wn^2 / (s^2 + 2 * dr * wn * s + wn^2);
F = (1 + L) * T / L;

figure(1); margin(L)
figure(2); nyquist(L)
bandwidth(F * L / (1 + L))
stepinfo(z_sys);

% sys = ss(L)
z_sys = ss(F * L / (1 + L));
stepinfo(z_sys)

%%

% X SYSTEM
wn = 0.8;
dr = 1.05;

kp = 18;
kd = 9;
ki = 2.5;

% wn = 0.6;
% dr = 1.1;

% kp = 12;
% kd = 7;
% ki = 3;

C = kp + ki / s + kd * s;
P = 1 / s^2;
L = C * P;
T = wn^2 / (s^2 + 2 * dr * wn * s + wn^2);
F = (1 + L) * T / L;

figure(1); margin(L)
figure(2); nyquist(L)
bandwidth(F * L / (1 + L))
x_sys = ss(F * L / (1 + L));
stepinfo(x_sys)

%%

pq = 18;
pw = 24;

C = pq + pw * s;
P = 1 / s^2;
L = C * P;
figure(1); margin(L)
figure(2); nyquist(L)
bandwidth(L / (1 + L))
p_sys = ss(L / (1 + L));
stepinfo(p_sys);

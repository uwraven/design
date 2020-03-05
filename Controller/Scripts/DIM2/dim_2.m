clear; clc;

% Vehicle properties
l = 1.5;
m_wet = 15; % kg
m_dry = 10; % kg
m_flow = 0.0008185; % kg / s / N
J = 1/12 * (l)^2;

% Control gains�
% kp ki kd
kr1 = [0 0 0];
kr2 = [0 0 0];
kth = [0.212 0.15 15];
K = [ kr1; kr2; kth ];

% Actuator properties
L = [1.0 0.5];
% Actuator limits
ELIM = [0.6 1.3] * m_wet * 9.81;
RLIM = [-15 15];
GLIM = deg2rad([-10 10]);

global t_prev total_err; t_prev = 0; total_err = [0 0 0];

% X = [r1 r2 v1 v2 th dth mass]
X_initial = [0 0 0 0 deg2rad(10) 0 m_wet];
Xd = [0 0 0 0 0 0];
tspan = 0:0.0001:50;

of = @(t, X) odefunc(t, X, Xd, K, L, ELIM, RLIM, GLIM, J, m_flow);

[t, X] = ode23(of, tspan, X_initial);

figure(1); clf;
plot(t, X(:,2)); hold on;
plot(t, X(:,4));
title('Y'); grid on;

figure(2); clf;
plot(t, X(:,1)); hold on;
plot(t, X(:,3));
title('X'); grid on;

figure(3); clf;
plot(t, X(:,5)); hold on;
plot(t, X(:,6));
title('\theta'); grid on;

figure(4); clf;
plot(t, X(:,7)); hold on;
plot(t, X(:,7));
title('m'); grid on;



function dX = odefunc(t, X, Xd, K, L, ELIM, RLIM, GLIM, J, m_flow)
global t_prev total_err;

% Get err from measured states
err = [Xd(1) - X(1), Xd(2) - X(2), Xd(5) - X(5)];
% change in error is equal to the change in X if desired X is const
d_err = -[X(3) X(4) X(6)];
total_err = total_err + err * (t - t_prev);

% Compute control gains (forces / torques)
k = K;
k(1,:) = k(1,:) * X(7);
k(2,:) = k(2,:) * X(7);
k(3,:) = k(3,:) * (X(7) * J);
u_r1 = k(1,1) * err(1) + k(1,2) * total_err(1) + k(1,3) * d_err(1); % F_x
u_r2 = k(2,1) * err(2) + k(2,2) * total_err(2) + k(2,3) * d_err(2); % F_y
u_r3 = k(3,1) * err(3) + k(3,2) * total_err(3) + k(3,3) * d_err(3); % T

controller_outputs = [ u_r1 u_r2 u_r3 ];

u_r2 = u_r2 + 9.81 * X(7);

% Allocate inputs

theta = X(5);
M = [cos(theta) sin(theta); -sin(theta) cos(theta)];
Fb = M * [u_r1; u_r2];

[Fe, Fr, g] = Allocation(Fb(1), Fb(2), u_r3, L(1), L(2));

% Fe = Fe + 9.81 * X(7);

allocation_outputs = [Fe Fr g];

% Clamp actuator inputs
% Fe = clamp(Fe, ELIM);
% Fr = clamp(Fr, RLIM);
% g = clamp(g, GLIM);

clamped_allocation_outputs = [Fe Fr g];

% Compute global frame forces on the vehicle
Fxg = cos(theta) * (Fr + Fe * sin(g)) - Fe * cos(g) * sin(theta);
Fyg = sin(theta) * (Fr + Fe * sin(g)) + Fe * cos(g) * cos(theta);
Tth = L(1) * Fe * sin(g) - L(2) * Fr; 

actuator_outputs = [ Fxg Fyg Tth ];

m = X(7);
Ji = J * m;
mdot = m_flow * Fe;

dX = zeros(7,1);
dX(1) = X(3); % velocity x
dX(2) = X(4); % velocity y
dX(3) = Fxg / m; % acc x
dX(4) = Fyg / m - 9.81; % acc y
dX(5) = X(6); % angular rate
dX(6) = Tth / Ji; % angular acc
dX(7) = -mdot; % change in vehicle mass

t_prev = t;

end


function [Fe, Fr, g] = Allocation(Fx, Fy, Tth, le, lr)

% V1.0
% F = Fy * cos(theta) - Fx * sin(theta);
% Fr = (Tth - Fx * le * cos(theta) - Fy * le * sin(theta)) / (le + lr);
% alpha = (Tth + Fx * lr * cos(theta) + Fy * lr * sin(theta)) / (le + lr);

% V2.0
g = atan(Tth / (Fy * (le - lr * Fx - lr)));
Fe = Fy / cos(g);
Fr = Fx - Fe * sin(g);

% allocation_outputs = [F Fr g]

end

function out = clamp(x, lim)
if x < lim(1)
	out = lim(1);
elseif x > lim(2)
	out = lim(2);
else
	out = x;
end
end
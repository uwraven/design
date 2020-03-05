clear; clc;

% Vehicle
l = 1.5;
m_wet = 15; % kg
m_dry = 10; % kg
m_flow = 0.0008185; % kg / s / N
J = 1/12 * (l)^2;

% Controller
% kr1 = [2.0 0.01 40];
kr1 = zeros(1,3);
kr2 = [15.0 0.04 110.0];
kth = [5.0 0.59 89];
K = [ kr1; kr2; kth ];

% Actuator
L = [1.0 1.0];
ELIM = [0.6 1.3] * m_wet * 9.81;
RLIM = [-15 15];
GLIM = deg2rad([-10 10]);
AS = [0 0 0];

% Integrator
X_initial = [0 2 0 0 0 0];
Xd = [0 0 0 0 0 0];
dt = 0.0001;
tspan = 0:dt:20;

t = tspan';
X = zeros(length(tspan), length(X_initial));
X(1,:) = X_initial;
U = zeros(length(tspan), 3);
U_global = zeros(length(tspan), 3);
U_shifted = zeros(length(tspan), 3);
M = zeros(length(tspan), 1);
M(1) = m_wet;
iErr = zeros(1, 3);

delay = [0.0 0.0 0.0];
delay_steps = int32(delay / dt);

LL = zeros(length(tspan), 7);

U_R3_PASS = 0;

% Perform integration
for i = 1:length(tspan)
	
    % Current state
    Xk = X(i, :);
	
	ts = t(i);

    % Proportional error
    pErr = [Xd(1)-Xk(1) Xd(2)-Xk(2) Xd(5)-Xk(5)];

    % Change in error
    dErr = -[Xk(3) Xk(4) Xk(6)];

    % Error integral
    iErr = iErr + pErr * dt;
	
	% Premultiply gains by vehicle factors
	k = K;
	k(1,:) = k(1,:) / M(i);
	k(2,:) = k(2,:) / M(i);
 	k(3,:) = k(3,:) / (M(i) * J);

    % Global frame control inputs
    u_r1 = k(1,1) * pErr(1) + k(1,2) * iErr(1) + k(1,3) * dErr(1);
    u_r2 = k(2,1) * pErr(2) + k(2,2) * iErr(2) + k(2,3) * dErr(2);
    u_r3 = k(3,1) * pErr(3) + k(3,2) * iErr(3) + k(3,3) * dErr(3);
	
    % Feedforward gravity offset
    u_r2 = u_r2 + M(i) * 9.81;
	
	U_global(i,:) = [u_r1 u_r2 u_r3];

    % Convert inputs to body frame
    theta = Xk(5);
    Mat = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    Fb = Mat * [u_r1; u_r2];
				
    % Allocate inputs
	
% 	a = 0.95;
% 	U_R3_PASS = U_R3_PASS * a + u_r3 * (1-a);
% 	NUM = 0;
% 	if abs(U_R3_PASS) < 1e-5
% 		NUM = 0;
% 	else
% 		NUM = U_R3_PASS;
% 	end
NUM = u_r3;

	DEN = Fb(2) * (L(1) - L(2) * Fb(1) - L(2));
	r = NUM / DEN;
	at = atan(NUM / DEN);
	at2 = atan2(NUM, DEN);
	
	LL(i,:) = [NUM DEN r at at2 Fb(1) Fb(2)];
	
	g = at;
	
	[g, AS(3)] = clamp(g, GLIM);

    Fe = Fb(2) / cos(g);
	
	[Fe, AS(1)] = clamp(Fe, ELIM);

    Fr = Fb(1) - Fe * sin(g);

    [Fr, AS(2)] = clamp(Fr, RLIM);
	
	U(i, :) = [Fe, Fr, g];
			
	if (i > delay_steps(1))
		Fe = U(i - delay_steps(1), 1); 
	else
		Fe = m_wet * 9.81;
	end
	if (i > delay_steps(2))
		Fr = U(i - delay_steps(2), 2); 
	else
		Fr = 0;
	end
	if (i > delay_steps(3))
		g = U(i - delay_steps(3), 3);
	else
		g = 0;
	end

	U_shifted(i,:) = [Fe, Fr, g];
	
    % Compute global frame forces
    Fxg = cos(theta) * (Fr + Fe * sin(g)) - Fe * cos(g) * sin(theta);
    Fyg = sin(theta) * (Fr + Fe * sin(g)) + Fe * cos(g) * cos(theta);
    Tth = L(1) * Fe * sin(g) - L(2) * Fr; 
	    
    % Update mass flow
    m = M(i);
    Ji = J * m;
    mdot = m_flow * Fe;

	% Break conditions
    if (i + 1) > length(tspan)
        break;
    elseif (M(i) < m_dry)
        break;
	elseif (abs(X(i,1)) > 5)
		break;
	end

    % Update state
    X(i+1,1) = X(i,1) + X(i,3) * dt + Fxg / m * dt * dt; % x + dx + dxx
    X(i+1,2) = X(i,2) + X(i,4) * dt + Fyg / m * dt * dt; % y + dy + dyy
    X(i+1,3) = X(i,3) + Fxg / m * dt; % dx + ddx
    X(i+1,4) = X(i,4) + (Fyg / m - 9.81) * dt; % dy + ddy
    X(i+1,5) = X(i,5) + X(i,6) * dt + Tth / Ji * dt * dt; % th + dth
    X(i+1,6) = X(i,6) + Tth / Ji * dt; % dth + ddth
    M(i+1) = M(i) - mdot * dt; % change in vehicle mass

end

figure(1); clf; plot(t, [X(:,1) X(:,3)]); legend('x', 'dx'); grid on;
figure(2); clf; plot(t, [X(:,2) X(:,4)]); legend('y', 'dy'); grid on;
figure(3); clf; plot(t, [X(:,5) X(:,6)]); legend('\theta', 'd\theta'); grid on;
figure(4); clf; plot(t, [U(:,1:2) U_shifted(:,1:2)]); legend('U_{Engine}', 'U_{RCS}'); grid on;
figure(5); clf; plot(t, [U(:,3) U_shifted(:,3)]); legend('\gamma_c', '\gamma_d'); grid on;
figure(6); clf; plot(t, U_global); legend('F_x', 'F_y', 'T'); grid on;
figure(7); clf; plot(t, LL(:,1:5)); legend('NUM', 'DEN', 'NUM/DEN', 'atan', 'atan2'); grid on;
figure(8); clf; plot(t, LL(:,6:7)); legend('fbx', 'fby'); grid on;




function [out, saturated] = clamp(x, lim)
    if x < lim(1)
        out = lim(1);
        saturated = 1;
    elseif x > lim(2)
        out = lim(2);
        saturated = 1;
    else
        out = x;
        saturated = 0;
    end
end
addpath 'rotations'

% Define geometry symbolically %

syms x y 'real'
syms lc 'real'      % distance from pivot to chamber mount in z dir
syms l1x l2x 'real' % x dir
syms l1y l2y 'real' % y dir
syms a b 'real'     % gimbal angles

% assume DCM is defined as alpha then beta (B*A*X)
M = dcm_y(b) * dcm_x(a);

% NOTE: the gimbal pivot defines the origin

% the x and y actuator vectors are defined from the vehicle mount to the
% combustion chamber mount. They are functions of the chamber fixed mounting
% distances l2x, l2y, and lc, as well as the vehicle fixed mount distance 
% l1x and l1y.

% actuator chamber mounting vectors
x_chamber_mount = [l2x 0 -lc]';
y_chamber_mount = [0 l2y -lc]';

% actuator vehicle mounting vectors
x_vehicle_mount = [l1x 0 0]';
y_vehicle_mount = [0 l1y 0]';

% chamber mount vectors are transformed by the desired gimbal angles
x_chamber_mount = M * x_chamber_mount;
y_chamber_mount = M * y_chamber_mount;

% actuator vectors
x_actuator = x_chamber_mount - x_vehicle_mount;
y_actuator = y_chamber_mount - y_vehicle_mount;

% actuator extensions are then given by the norm
x_extension = norm(x_actuator);
y_extension = norm(y_actuator);


%% Plot rates

aspace = deg2rad([-13:0.5:13]);
bspace = deg2rad([-13:0.5:13]);

x_eq = x == x_extension;
y_eq = y == y_extension;

r_lc = 0.2; % z distance from pivot to combustion chamber mount
r_1 = 0.1; % x and y distances from pivot to vehicle mount
r_2 = 0.05; % x and y distances from pivot to combustion chamber mount (e.g. combustion chamber outer radius <= 5cm)

x_neq = matlabFunction(subs(x_extension, [lc l1x l2x], [r_lc r_1 r_2]));
y_neq = matlabFunction(subs(y_extension, [lc l1y l2y], [r_lc r_1 r_2]));

% differentiate x actuator length with respect to beta angle
% this provides change in actuator extension vs gimbal angle
% multiplication by desired angular velocity provides required actuator velocity
dx_neq_0 = matlabFunction(diff(x_neq_0, b));

clf
fsurf(dx_neq_0, deg2rad([-20 20]))
zlabel("m / rad")
ylabel("\beta (rad)")
xlabel("\alpha (rad)")




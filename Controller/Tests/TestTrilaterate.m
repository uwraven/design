clc; clear;
addpath(genpath('..'));

r1 = 50;
r2 = 50;
r3 = 50;

U = [5 0 0];
V = [25 25 0];

[x y z] = trilaterate(r1, r2, r3, U(1), V(1), V(2))

figure(1); clf; hold on; grid on;
axis equal;

[X, Y, Z] = sphere();
surf(r1 * X, r1 * Y, r1 * Z, 'FaceAlpha', 0.5)

[X, Y, Z] = sphere();
surf(r2 * X + U(1), r2 * Y, r2 * Z, 'FaceAlpha', 0.5)

[X, Y, Z] = sphere();
surf(r3 * X + V(1), r3 * Y + V(2), r3 * Z, 'FaceAlpha', 0.5)

plot3(x, y, z, 'rx');
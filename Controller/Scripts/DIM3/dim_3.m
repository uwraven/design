clc; clear;

% Vehicle properties
le = 0.5;
lr = 1.0;
m = 15.0;

% Static commands
Fx = 9.81 * m;
Fy = 0;
Fz = 0;

% Mapped commands
My = -200:10:200;
Mz = -200:10:200;

% Gimbal inputs
g = @(My, Mz) atan(((Mz - Fy * le) / (lr - le) - Fy)/Fx) + My * 0;
b = @(My, Mz) atan((Fz - (My - Fz * le) / (lr - le)) / sqrt(((Mz - Fy * le) / (lr - le) - Fy)^2 + Fx^2));
% b = @(My, Mz, g) atan((Fz - (My - Fz * le) / (lr - le)) / Fx * cos(g));

G = zeros(length(My), length(Mz));
B = zeros(length(My), length(Mz));

for yi = 1:length(My)
	for zi = 1:length(Mz)
		G(yi, zi) = g(My(yi), Mz(zi));
		B(yi, zi) = b(My(yi), Mz(zi));
	end
end

figure(1); clf;
surf(My, Mz, G); hold on; grid on;
ts = surf(My, Mz, ones(length(My), length(Mz)) * deg2rad(10), 'FaceAlpha', 0.4);
tb = surf(My, Mz, -ones(length(My), length(Mz)) * deg2rad(10), 'FaceAlpha', 0.4);
ts.EdgeColor = 'none';
tb.EdgeColor = 'none';
xlabel('My');
ylabel('Mz');
zlabel('\gamma');

figure(2); clf;
surf(My, Mz, B); hold on; grid on;
ts = surf(My, Mz, ones(length(My), length(Mz)) * deg2rad(10), 'FaceAlpha', 0.4);
tb = surf(My, Mz, -ones(length(My), length(Mz)) * deg2rad(10), 'FaceAlpha', 0.4);
ts.EdgeColor = 'none';
tb.EdgeColor = 'none';
xlabel('My');
ylabel('Mz');
zlabel('\beta');


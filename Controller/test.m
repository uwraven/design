addpath('Core');

q = [1 0 0 0]'

w = @(dt) [1 2 4]' + [0.01 0.01 0.005]' * dt;

dt = 0.001;
ts = 0:dt:10;

tic

for i = 1:length(ts)
	q = Quaternion.rk4N(q, w, dt);
end

toc

q

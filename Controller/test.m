addpath('Core');

dt = 0.001;
ts = 0:dt:10;

q = zeros(length(ts), 7);
q(1,:) = [1 0 0 0 0 0 0];

tic

for i = 1:length(ts)-1
	q(i+1, :) = Quaternion.integrateAttitude(q(i, :)', [1 0 0]', dt);
end

toc

plot(ts, q(:,1:4))
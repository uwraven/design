clc; clear;

RCS = ReactionControl(15, 1.0, 0, 1, 0);

RCS.setCommands([0 0 0 0 0 0 0]);

dt = 0.0001;
ts = 1:dt:10;
O = zeros(length(ts), 8);

for i = 1:length(ts)
	R = 18 * sin(i * dt * 5);
	RCS.setCommands([R 0 0 0 0 0 0]);
	RCS.tick(dt);
	[F, M, y] = RCS.getForceOnThisTick();
	O(i,:) = [F M, y, R];
end

figure(1); clf; plot(ts, [O(:,1) O(:,7:8)]); grid on;


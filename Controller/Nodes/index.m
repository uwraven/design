clc; clear;

RCS = ReactionControl(15, 1.0, 0, 1, 0);

RCS.setCommands([0.01 0 0 0 2 0 0]);

ts = 1:0.0005:10;
O = zeros(length(ts), 8);

for i = 1:length(ts)
	RCS.tick(0.001);
	R = 20 * cos(i / 100);
	RCS.setCommands([R 0 0 0 0 0 0]);
	[F, M, y] = RCS.getForceOnThisTick();
	O(i,:) = [F M, y, R];
end

plot(ts, [O(:,1) O(:,8)])


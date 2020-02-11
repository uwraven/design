clear; clc;
addpath('Core');
addpath('Nodes');

node = Node("test", true);
node.timer.setInterval(10);
sns = Sensor(0.0, 10);

clock = Clock("ms");

for i = 1:100
	clock.tick()
	t = clock.getElapsedTime();
	sns.timer.tick(t)
end

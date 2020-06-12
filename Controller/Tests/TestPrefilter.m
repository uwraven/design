clear; clc;

wn = 2.0;
dr = 1.1;

kp = 0.02;
ki = 0;
kd = 0.8;

prefilter = Prefilter();
prefilter.setGains(wn, dr, kp, ki, kd);
prefilter.setInitialValue(5);
prefilter.setTarget(10)

dt = 0.001;
f = 10;
ts = 0:dt:f;

X = zeros(length(ts), 1);

for i = 1:length(ts)
    prefilter.update(dt);
    X(i) = prefilter.output;
end

plot(ts, X);
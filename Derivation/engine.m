s = tf('s');

tau = 0.5

F = 1 / (tau * s + 1)

sys = ss(F)
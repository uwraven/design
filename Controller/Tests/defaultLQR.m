function K = defaultLQR()

[A, B] = LinearizedNormalDynamics();
Q = diag(bryson([2 2 1 0.5 0.5 0.8 0.1 0.1 0.1 0.05 0.05 0.05]));
R = diag(bryson([1 1 1 1 1 1]));
K = lqr(A, B, Q, R);

end


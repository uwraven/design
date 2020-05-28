function K = defaultLQR(m, J)

[A, B] = LinearizedNormalDynamics(m, J);
Q = diag(bryson([0.5 1 1 0.5 1 1 1 1 1 1 1 1]));
R = diag(bryson([1 1 1 1 1 1]));
K = lqr(A, B, Q, R);

end


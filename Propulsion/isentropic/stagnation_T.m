function T0 = stagnation_T(T, k, M)
T0 = T .* (1 + 0.5 * (k - 1) .* M.^2);
end
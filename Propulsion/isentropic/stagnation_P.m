function P0 = stagnation_P(p, k, M)
P0 = p * (1 + 0.5 * (k - 1) * M^2) ^ (k / (k - 1));
end

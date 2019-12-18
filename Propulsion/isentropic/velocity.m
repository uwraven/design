function v = velocity(k, R, T0, P, P0)
v = sqrt(2 * k / (k - 1) * R * T0 * (1 - (P / P0) ^ ((k - 1) / k)));
end


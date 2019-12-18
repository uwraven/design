function m_dot = mdot(At, P0, k, R, T0)
m_dot = At * P0 * k * sqrt((2 / (k + 1))^((k + 1) / (k - 1)) / (k * R * T0));
end
function M = mach_number(k, T, T0)
M = sqrt(2 / (k - 1) .* (T0 ./ T - 1));
end
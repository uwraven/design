function e = matched_a_ratio(p_0, p_e, k)

e = 1 / (((k + 1) / 2) .^ (1 / (k - 1)) .* (p_e / p_0) .^ (1 / k) .* sqrt((k + 1) ./ (k - 1) * (1 - (p_e ./ p_0) .^ ((k - 1) ./ k))));

end
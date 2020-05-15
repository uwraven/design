function [x, y, z] = trilaterate(r1, r2, r3, U, Vx, Vy)
    % Computes the intersection of three spheres
    % Inputs: sphere radii, x / y offsets
    % Assume spheres can be expressed by the following eq:
    % r1^2 = x^2 + y^2 + z^2
    % r2^2 = (x - U)^2 + y^2 + z^2
    % r3^2 = (x - Vx)^2 + (y - Vy)^2 + z^2

    x = (r1^2 - r2^2 + U^2) / (2 * U);
    y = (r1^2 - r3^2 + norm([Vx Vy])) - 2 * Vx * Vy / (2 * Vy);
    z = sqrt(r1^2 - x^2 - y^2);

end
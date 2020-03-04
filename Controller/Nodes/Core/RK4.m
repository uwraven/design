function X = RK4(f, x, dt)
    % A general RK4 integration method
    % Requires a state transition function f which accepts X and dt as arguments
    % http://ancs.eng.buffalo.edu/pdf/ancs_papers/2013/geom_int.pdf
    % https://lpsa.swarthmore.edu/NumInt/NumIntFourth.html

    k1 = dt * f(x, 0);
    k2 = dt * f(x + k1 / 2, dt / 2);
    k3 = dt * f(x + k2 / 2, dt / 2);
    k4 = dt * f(x + k3, dt);

    X = x + 1 / 6 * (k1 + 2 * k2 + 2 * k3 + k4);

end

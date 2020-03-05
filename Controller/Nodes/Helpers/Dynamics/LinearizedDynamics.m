function [A, B] = LinearizedDynamics(m, I)
    % Wrapper to compute linearized dynamics model for a quaternion parameterized rocket

    syms r1 r2 r3 v1 v2 v3 q0 q1 q2 q3 w1 w2 w3 'real';
    syms Fx Fy Fz Mx My Mz 'real';

    % State
    r = [r1 r2 r3]';    % global position
    v = [v1 v2 v3]';    % global velocity
    q = [q0 q1 q2 q3]'; % attitude quaternion
    w = [w1 w2 w3]';    % angular rates

    % Inputs
    F = [Fx Fy Fz]';
    M = [Mx My Mz]';

    dr = v;
    dv = 1 / m * F;
    dq = 1 / 2 * skew4(w) * q;
    dw = I^-1 * (M - cross(w, I * w));

    x = [
        r
        v
        q(2:4)
        w
    ];

    u = [
        F
        M
    ];

    dx = [
        dr
        dv
        dq(2:4)
        dw
    ];

    JA = jacobian(dx, x);
    JB = jacobian(dx, u);

    x_eq = [0 0 0 0 0 0 0 0 0 0 0 0];
    u_eq = [0 0 0 0 0 0];

    A = double(subs(JA, [x' u' q0], [x_eq u_eq 1]));
    B = double(subs(JB, [x' u' q0], [x_eq u_eq 1]));

end
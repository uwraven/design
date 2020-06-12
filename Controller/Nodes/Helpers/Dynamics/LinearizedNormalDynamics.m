function [A, B] = LinearizedDynamics()
    % Wrapper to compute linearized dynamics model for a quaternion parameterized rocket

    syms r1 r2 r3 v1 v2 v3 q0 q1 q2 q3 w1 w2 w3 'real';
    syms alx aly alz aax aay aaz 'real';

    % State
    r = [r1 r2 r3]';    % global position
    v = [v1 v2 v3]';    % global velocity
    q = [q0 q1 q2 q3]'; % attitude quaternion
    w = [w1 w2 w3]';    % angular rates

    % Inputs
    al = [alx aly alz]';
    aa = [aax aay aaz]';

    dr = v;
    dv = al;
    dq = 1 / 2 * Quaternion.productArr(q, [0; w]);
    dw = aa;

    x = [
        r
        v
        q(2:4)
        w
    ];

    u = [
        al
        aa
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
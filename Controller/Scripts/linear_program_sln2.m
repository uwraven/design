
% U = [ Fex Fey Fez Fr1 Fr2 Fr3 ];

% Lims [
%   Fexmin  Fexmax
%   Feymin  Feymax
%   Fezmin  Fezmax
%   Fr1min  Fr1max
%   Fr2min  Fr2max
%   Fr3min  Fr3max
% ]

Ur = 100 * [ 
    0.279120121959425
    -0.002943000000000
    -1.309048230235206
    0.002000000000000
    0.020000000000000
    0.000000000000000
];

lb = 100 * [
    -0.167245541228246
    -0.167245541228246
    -1.600000000000000
    -0.100000000000000
    -0.100000000000000
    -0.100000000000000
    0
];

ub = [
    16.724554122824557
    16.724554122824557
    -90.000000000000000
    10.000000000000000
    10.000000000000000
    10.000000000000000
    1.000000000000000
];

Fex = optimvar('Fex', 'LowerBound', lb(1), 'UpperBound', ub(1));
Fey = optimvar('Fey', 'LowerBound', lb(2), 'UpperBound', ub(2));
Fez = optimvar('Fez', 'LowerBound', lb(3), 'UpperBound', ub(3));
Fr1 = optimvar('Fr1', 'LowerBound', lb(4), 'UpperBound', ub(4));
Fr2 = optimvar('Fr2', 'LowerBound', lb(5), 'UpperBound', ub(5));
Fr3 = optimvar('Fr3', 'LowerBound', lb(6), 'UpperBound', ub(6));
k = optimvar('k', 'LowerBound', 0, 'UpperBound', 1);

prob = optimproblem('Objective', k, 'ObjectiveSense', 'min');
prob.Constraints.c1 = Fex + k * Ur(1) == Ur(1);
prob.Constraints.c2 = Fey + k * Ur(2) == Ur(2);
prob.Constraints.c3 = Fez + k * Ur(3) == Ur(3);
prob.Constraints.c4 = Fr1 + k * Ur(4) == Ur(4);
prob.Constraints.c5 = Fr2 + k * Ur(5) == Ur(5);
prob.Constraints.c6 = Fr3 + k * Ur(6) == Ur(6);

problem = prob2struct(prob);

[sol, fval, exitflag, output] = linprog(problem);
sol
fval
exitflag
output
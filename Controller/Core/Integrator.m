classdef Integrator < handle

properties (Access = public)
    dt double = 0.001
    tf double = 30
    transitionFunction handle
    breakFunction handle
end

properties (Access = private)

end

methods (Access = public)
    function self = Integrator() 
	end

    function [t, X, U, O] = run(self, tspan, X_0)
        ts = length(tspan);
        t = tspan;
        X = zeros(ts, length(X_0));

        for ii = 1:ts

            % Get the next state, the user is responible for the entire state transition
            % This allows for a nonlinear state transition (ex: quaternion rotation)
            [X_i, U_i, O_i] = self.transitionFunction(t, X, U);

            % During the first loop, set the sizes of U and O
            if ii == 1
                U = zeros(ts, length(U_i));
                O = zeros(ts, length(O_i));
            end

            X(ii, :) = X_i;
            U(ii, :) = U_i;

            if self.breakFunction(X)
                return;
            end
        end
	end
end

methods (Access = private)
    
end

end
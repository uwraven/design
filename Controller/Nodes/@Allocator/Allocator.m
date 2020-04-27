classdef Allocator < handle

properties (Access = public)
    saturate = false
    limits
    clamped
end

properties (GetAccess = public, SetAccess = private)
    L
    H
    Hc
end

methods (Access = public)
    function self = Allocator()
    end

    function self = setActuatorLimits(self, limits)
        self.limits = reshape(limits, 6, 2);
    end

    function self = setActuatorArms(self, L)

        self.L = L;

        L1 = L(1);
        L2 = L(2);
        L3 = L(3);

        self.H = [
            1 0 0 -1 -1 0 0 1 1
            0 1 0 0 0 -1 0 0 1
            0 0 1 0 0 0 0 0 0 
            0 -L1 0 0 0 -L2 0 0 L2
            L1 0 0 L2 L2 0 0 -L2 -L2
            0 0 0 -L3 L3 0 0 -L3 L3
        ];

        self.Hc = inv(self.H);

    end

    function U = nonlinearAllocation(self, Ur)

    end

    function U = linearAllocation(self, Ur)
        % U is a vector [Fx Fy Fz Mx My Mz] in body frame

        % Solve the linear EOM
        U = self.Hc * Ur;

        % Dumb actuator clamping for now
        % TODO: implement allocation strategy
        if (self.clamped)
            for i = 1:6
                U(i) = clamp(self.limits(i, :), U(i));
            end
        end

    end
end

end
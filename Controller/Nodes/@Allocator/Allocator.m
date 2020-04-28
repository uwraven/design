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
        % Expects 9x1 vector
        self.limits = limits;
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
        % Ur = [Fx Fy Fz Mx My Mz];

        B = atan(-Ur(1), Ur(3));
        G = atan(-Ur(2) * sin(B), Ur(1));
        Fe = -Ur(2) / sin(G);
        FRx = self.L(1) / self.L(2) * Fe * cos(G) * sin(B) - 1 / self.L(2) * Ur(5);
        FRy = -self.L(1) / self.L(3) * Fe * sin(G) + 1 / self.L(2) * Ur(4);
        FRz = Ur(6);

        U = [B G Fe FRx FRy FRz];

    end

    function U = linearAllocation(self, Ur)
        % Ur = [Fx Fy Fz Mx My Mz];

        % Solve the linear EOM
        U = self.Hc * Ur;

    end

    function U = allocate(self, Ur)

        U = linearAllocation(Ur);

        saturated = self.saturation(U);
    
        if (~saturated) 
            return
        else
            % Allocation strategy...
            return
        end
    end

    function saturated = saturation(self, U)
        saturated = false;
        for i = 1:length(U)
            if (self.limits(i, 1) < U && self.limits(i, 2) > U)
                saturated = true;
            end
        end
    end

end

end
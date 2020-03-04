classdef Allocator < handle

properties (Access = public)
    saturate = false
    limits
    L_E
    L_R1
    L_R2
    clamped
end

methods (Access = public)
    function self = Allocator()
    end

    function self = setActuatorLimits(self, limits)
        self.limits = reshape(limits, 6, 2);
    end

    function self = setActuatorGeometry(self, l_e, lr1, lr2)
        self.L_E = l_e;
        self.L_R1 = lr1;
        self.L_R2 = lr2;
    end

    function U = allocate(self, Ur)
        % U is a vector [Fx Fy Fz Mx My Mz] with F in vehicle frame

        % Solve nonlinear EOM
        F_RY = (Ur(6) - Ur(2) * self.L_E) / (self.L_R1 - self.L_E);
        F_RP = (Ur(5) - Ur(3) * self.L_E) / (self.L_R1 - self.L_E);
        F_RR = 1 / (2 * self.L_R2) * Ur(4);

        G = atan((F_RY - Ur(2)) /  Ur(1));
        B = atan(((Ur(3) - F_RP) * cos(G)) / Ur(1));
        F_E = Ur(1) / (cos(G) * cos(B));

        U = [G B F_E F_RY F_RP F_RR];

    end

    function U = allocateLinear(self, Ur)
        % U is a vector [Fx Fy Fz Mx My Mz] in body frame

        % Solve the linear EOM
        U = [
            Ur(1)
            (Ur(6) - Ur(2) * self.L_R1) / (self.L_E - self.L_R1)
            (Ur(5) - Ur(3) * self.L_R1) / (self.L_E - self.L_R1)
            Ur(4) / (2 * self.L_R2)
            (Ur(3) * self.L_E - Ur(5)) / (self.L_E - self.L_R1)
            (Ur(2) * self.L_E - Ur(6)) / (self.L_E - self.L_R1)
        ];
        % [Fex Fey Fez Frr Frp Fry]

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
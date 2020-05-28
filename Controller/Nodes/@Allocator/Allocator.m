classdef Allocator < handle

properties (Access = public)
    saturate = false
    enabled = false
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

    function update(self, dt)
        
    end

    function self = setActuatorLimits(self, limits)
        self.limits = reshape(limits, 9, 2);
    end

    function self = setActuatorArms(self, L)

        self.L = L;

        L1 = L(1);      % Arm from COM to engine
        L2 = L(2);      % Arm from COM to rcs
        L3 = L(3);      % Arm from rcs center to thruster

        self.H = [
            1 0 0 -1 -1 0 0 1 1
            0 1 0 0 0 -1 0 0 1
            0 0 1 0 0 0 0 0 0 
            0 -L1 0 0 0 -L2 0 0 L2
            L1 0 0 L2 L2 0 0 -L2 -L2
            0 0 0 -L3 L3 0 0 -L3 L3
        ];

        self.Hc = pinv(self.H);

    end

    function U = nonlinearAllocation(self, Ur)
        
    end

    function U = clampedLinearAllocation(self, Ur)
        % Ur is a vector [alx aly alz aax aay aaz] in body frame
        % U is a vector of actuator inputs

        % Solve the linear EOM
        U = self.Hc * Ur;

        % Dumb actuator clamping for now
        % TODO: implement allocation strategy
        % if (self.clamped)
        %     for i = 1:9
        %         U(i) = clamp(self.limits(i, :), U(i));
        %     end
        % end

    end
end

end
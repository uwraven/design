classdef Allocator < handle

properties (Access = public)
    saturate = false
    enabled = false
    limits
    modes = struct('free', 0, 'clamped', 1, 'projected', 2, 'optimal', 3);
    mode;
end

properties (GetAccess = public, SetAccess = private)
    L
    H
    Hc
end

properties (Access = private)
    options
end

methods (Access = public)
    function self = Allocator()
        self.options = optimoptions('linprog', 'Algorithm', 'dual-simplex', 'Display', 'none');
        self.mode = self.modes.free;
    end

    function update(self, dt)
        
    end

    function self = setActuatorLimits(self, limits)
        self.limits = reshape(limits, 6, 2);
    end

    function self = setActuatorArms(self, L)

        self.L = L;

        L1 = L(1);      % Arm from COM to engine
        L2 = L(2);      % Arm from COM to rcs
        L3 = L(3);      % Arm from rcs center to thruster

        % Uncondensed rcs
        % self.H = [
        %     1 0 0 -1 -1 0 0 1 1
        %     0 1 0 0 0 -1 0 0 1
        %     0 0 1 0 0 0 0 0 0 
        %     0 -L1 0 0 0 -L2 0 0 L2
        %     L1 0 0 L2 L2 0 0 -L2 -L2
        %     0 0 0 -L3 L3 0 0 -L3 L3
        % ];

        % Condense rcs forces
        self.H = [
            1   0   0   1   1   0
            0   1   0   0   0   1
            0   0   1   0   0   0
            0   -L1 0   0   0   L2
            L1  0   0   -L2 -L2 0
            0   0   0   -L3 L3  0
        ];

        self.Hc = pinv(self.H);

    end

    function U = allocate(self, Ur)
        % Allocate inputs
        U = zeros(6, 1);
        switch self.mode
            case self.modes.free
                U = self.freeAllocation(Ur);
            case self.modes.clamped
                U = self.clampedLinearAllocation(Ur);
            case self.modes.projected
                U = self.projectionLinearAllocation(Ur);
            case self.modes.optimal
                U = self.optimalLinearAllocation(Ur);						
        end
    end

    function U = freeAllocation(self, Ur)
        U = self.Hc * Ur;
    end


    function U = clampedLinearAllocation(self, Ur)
        % Ur is a vector [alx aly alz aax aay aaz] in body frame
        % U is a vector of actuator inputs
        % Solve the linear EOM
        U = self.Hc * Ur;

        % Dumb actuator clamping for now
        % TODO: implement allocation strategy
        for i = 1:length(U)
            % U(i) = clamp(self.limits(i, :), U(i))
            U(i) = clamp(self.limits(i, :), U(i));
        end
    end

    function U = projectionLinearAllocation(self, Ur)
        U = self.Hc * Ur;
        input_eps = 1; 

        for i = 1:length(U)
            if abs(U(i)) > 0
                ep = 1;
                if U(i) < self.limits(i,1)
                    ep = abs(self.limits(i,1) / U(i));
                elseif U(i) > self.limits(i, 2)
                    ep = abs(self.limits(i, 2) / U(i));
                end
                if ep < input_eps
                    input_eps = ep;
                end
            end
        end

        U = input_eps * U;

    end

    function U = optimalLinearAllocation(self, Ur)
        Ur
        U = self.Hc * Ur
        
        if self.isOutOfBounds(U)
            % Solve linear program
            % Objective function minimizes RCS input and error between resultant and requested
            % Formulate inequalities based on request and limits

            Uaxy = self.xyAllocation(Ur(4), Ur(5))
            Fz = self.zAllocation(Uaxy, Ur(3));

            f = [
                zeros(6, 1)
                1
            ];

            Aeq = [ diag(ones(length(U), 1)) Ur];

            beq = Ur;

            lb = [
                self.limits(:,1)
                0
            ];

            ub = [
                self.limits(:,2)
                1
            ];

            ua = linprog(f, [], [], Aeq, beq, lb, ub, self.options);

            if (length(ua) == 0)
                U = zeros(6,1);
                U = self.clampedLinearAllocation(Ur);
            else
                U = reshape(ua(1:6), 6, 1);
            end
        end

    end
end

methods (Access = private)
    function outOfBounds = isOutOfBounds(self, Ur)
        for i = 1:length(Ur)
            if Ur(i) < self.limits(i,1) || Ur(i) > self.limits(i,2)
                outOfBounds = true;
                break
            else
                outOfBounds = false;
            end
        end
    end

    function Uo = xyAllocation(self, Ur, U)
        % J = [ Fex Fey Fr1 Fr2 Fr3 Xi ]
        f = [ zeros(6, 1) 1 ]
        A = [
            1 0 0 0 0 0 0
            0 1 0 0 0 0 0
            0 0 1 0 0 0 0
            0 0 0 1 0 0 0
            
        ]
        Aeq = [ self.H(4:5,:) [Ur(4); Ur(5)] ]
        beq = [Tx; Ty]
        lb = [self.limits(:,1); 0];
        ub = [self.limits(:,2); 1];
        [sol,fval,exitflag,output] = linprog(f, [], [], Aeq, beq, lb, ub, self.options)
        Uo = self.Hc * sol(1:6)
        error()
    end

    function Uo = zAllocation(self, Ua, Fz)
        Uo = Fz;
    end
end

end
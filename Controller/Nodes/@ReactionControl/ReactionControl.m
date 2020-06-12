classdef ReactionControl < handle

properties (Access = public)
    continuous = false
end

properties (Access = private)
    thrusters = [];
end

properties (SetAccess = private, GetAccess = public)
    localizedResultant = zeros(6, 1);
end

methods (Access = public)
    function self = ReactionControl()
        
    end

    function update(self, dt)
        % Resultant of form [Fx Fy Fz Mx My Mz]
        % For each pair of thrusters:
        % - compute the next step in time
        % - if the PWM controller output is active, then solenoid is open
        % - from thruster position and direction, compute the resulting forces and moments
        % - add these to the sum of forces and moments from all thrusters
        self.localizedResultant = zeros(6, 1);
        for i = 1:length(self.thrusters)
            thruster = self.thrusters(i);
            thruster.update(dt);
            F = zeros(3, 1);
            if (self.continuous)
                F = -thruster.direction * thruster.pwm.reference;
            elseif (thruster.firing)
                F = -thruster.direction * thruster.thrust;
            end
            self.localizedResultant = self.localizedResultant + [
                F,
                cross(thruster.position, F)
            ];
        end
    end

    function setAllocation(self, U)
        % U = [ Fr1' Fr2' Fr3' ]
        % Convert from primed inputs to signed inputs
        u = self.getSignedInputs(U);

        for i = 1:length(self.thrusters)
            self.thrusters(i).pwm.reference = u(i);
        end
    end

    function u = getSignedInputs(self, U)
        % U = [Fr1' Fr2' Fr3']
        % Convert to [Fr1 Fr2 Fr3 Fr4 Fr5 Fr6]
        u = zeros(6, 1);
        
        if (U(1) > 0)
            u(4) = abs(U(1));
        else
            u(2) = abs(U(1));
        end

        if (U(2) > 0)
            u(5) = abs(U(2));
        else
            u(1) = abs(U(2));
        end

        if (U(3) > 0)
            u(6) = abs(U(3));
        else
            u(3) = abs(U(3));
        end

        return;
    end

    function setThrusters(self, thrusters)
        self.thrusters = thrusters;
    end
end

end
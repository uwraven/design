classdef ReactionControl < handle

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
            if (thruster.firing)
                F = -thruster.direction * thruster.thrust;
                self.localizedResultant = self.localizedResultant + [
                    F,
                    cross(thruster.position, F)
                ];
            end
        end
    end

    function setAllocation(self, U)
        for i = 1:length(self.thrusters)
            self.thrusters(i).pwm.reference = U(i);
        end
    end

    function setThrusters(self, thrusters)
        self.thrusters = thrusters;
    end
end

end
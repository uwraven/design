classdef ColdGasThruster < handle

properties (Access = public)
    pwm PWMController
end

properties (Access = private)
end

properties (SetAccess = private, GetAccess = public)
    impulseDuration
    position
    direction
    thrust
end

methods (Access = public)
    function self = ColdGasThruster(nominalThrust)
        self.position = [0 0 0]';
        self.direction = [0 0 0]';
        self.thrust = nominalThrust;
        self.impulseDuration = 0.05;
        self.pwm = PWMController(self.impulseDuration, nominalThrust);
    end

    function update(self, dt)
        self.pwm.update(dt);
    end

    function place(self, P, D)
        % position is some 3 vector
        % direction describes the normal vector of RCS exit plane (i.e. opposite of resultant thrust)
        self.position = P;
        self.direction = D;
    end

    function [F, M] = getForceOnThisupdate(self)
        % Force acts in opposite direction of nozzle exit plane normal vector
        F = -self.direction' * self.thrust;
        % Compute the cross product to get resultant moments about cg
        M = cross(-self.direction' * self.thrust, self.position');
    end
end

end
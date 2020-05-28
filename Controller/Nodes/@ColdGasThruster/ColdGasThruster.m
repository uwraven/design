classdef ColdGasThruster < handle

properties (Access = public)
    pwm PWMController
    firing = false;
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
    function self = ColdGasThruster(nominalThrust, P, D)
        self.position = reshape(P, 3, 1);
        self.direction = reshape(D, 3, 1);
        self.thrust = nominalThrust;
        self.impulseDuration = 0.05;
        self.pwm = PWMController(self.impulseDuration, nominalThrust);
    end

    function update(self, dt)
        self.pwm.update(dt);
        self.firing = self.pwm.output > 0;
    end
end

end
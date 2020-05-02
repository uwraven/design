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
    function self = ColdGasThruster(nominalThrust)
        self.position = [0 0 0]';
        self.direction = [0 0 0]';
        self.thrust = nominalThrust;
        self.impulseDuration = 0.05;
        self.pwm = PWMController(self.impulseDuration, nominalThrust);
    end

    function update(self, dt)
        self.pwm.update(dt);
        self.firing = pwm.output > 0;
    end

    function place(self, P, D)
        % position is some 3 vector
        % direction describes the normal vector of RCS exit plane (i.e. opposite of resultant thrust)
        self.position = P;
        self.direction = D;
    end
end

end
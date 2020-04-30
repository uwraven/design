classdef LinearActuator < handle

properties
    controller;
    enableAntiWindup = true;
    targetExtension = 0;
end

properties (GetAccess = public, SetAccess = private)
    x = 0;
    lims = [0 100];
end
methods
    function self = LinearActuator()
        self.controller = PIDController(1, 0, 0);
    end

    function update(self, dt)
        % compute tracking error
        u = self.targetExtension - self.x;
        saturated = x > lims(1) && x < lims(2);
        self.controller.update(dt, u, saturated);
    end

    function setLimits(self, lims)
        self.lims = lims;
    end

end


end
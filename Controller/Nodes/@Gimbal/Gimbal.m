classdef Gimbal < handle

properties (Access = public)
    actuatorGamma
    actuatorBeta
    range
end

methods
    function self = Gimbal()
        self.actuatorGamma = LinearActuator();
        self.actuatorBeta = LinearActuator();
        self.range = deg2rad(2);
    end

    function update(self, dt)
        self.actuatorGamma.update(dt);
        self.actuatorBeta.update(dt);
    end
end

methods (Access = private)
    
end

end
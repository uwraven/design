classdef Gimbal < handle

properties
    actuatorGamma
    actuatorBeta
end

methods
    function self = Gimbal()
        actuatorGamma = LinearActuator()
        actuatorBeta = LinearActuator()
    end

    function update(dt)
    end
end

methods (Access = private)
    
end

end
classdef Gimbal < handle

properties (Access = public)
    actuatorGamma
    actuatorBeta
end



methods
    function self = Gimbal()
        actuatorGamma = LinearActuator();
        actuatorBeta = LinearActuator();
    end

    function update(dt)
        actuatorGamma.update(dt);
        actuatorBeta.update(dt);
    end
end

methods (Access = private)
    
end

end
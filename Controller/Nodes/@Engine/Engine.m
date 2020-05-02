classdef Engine < handle

properties 
    gimbal
end

methods
    function self = Engine()
        self.gimbal = Gimbal()
    end

    function setReference(self, reference)
        % Fe is the linear approximation of engine forces requested by the allocator (R3)
        % Fex Fey Fez

        % First, convert to proper nonlinear values
        [Fe, gamma, beta] = linearInputs(reference);

    end

    function update(self, dt)

    end
end

methods (Access = private)
    function [Fe, gamma, beta] = linearInputs(self, requested)
        Fe = -requested(3);
        gamma = -requested(2) / Fe;
        beta = -requested(1) / Fe;
    end

    function [Fe, gamma, beta] = nonlinearInputs(self, requested)
        beta = atan(requested(1), requested(3));
        gamma = atan(-requested(2) * sin(beta), requested(1));
        Fe = -requested(2) / sin(gamma);
    end
end

end
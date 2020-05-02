classdef EngineAssembly < handle

properties 
    gimbal
    thrust
    specificThrust % inverse of thrust specific mass flow
    engineArm = 0.1;
    gimbalArm = 0.2;
end

methods
    function self = EngineAssembly()
        self.gimbal = Gimbal()
    end

    function setTarget(self, reference)
        % Reference (Fe) is the linear approximation of engine forces requested by the allocator
        % Fex Fey Fez
 
        % First, convert to proper nonlinear values
        [Fe, g, b] = nonlinearInputs(reference);

        % Compute resultant engine pointing vector
        Ev = engineArm * [
            sin(g) * cos(b)
            sin(b)
            cos(g) * cos(b)
        ];

        % Compute required actuator extensions from difference between
        % engine pointing vector and gimbal mounting vectors
        self.gimbal.actuatorBeta.setTarget(norm(Ev - [self.gimbalArm 0 0]'));
        self.gimbal.actuatorGamma.setTarget(norm(Ev - [0 self.gimbalArm 0]));
    end

    function update(self, dt)
        % Engine is assumed to produce thrust ideally with no delay
        % TODO:: Add engine plant model + thrust controller
        self.gimbal.update(dt);

        % Get engine direction from actuator extensions
        % This function does not account for the radial extension from
        % the lower engine mount to linear actuator
        [x, y, z] = trilaterate(...
            self.engineArm,...
            self.gimbal.actuatorGamma.x,...
            self.gimbal.actuatorBeta.x,...
            self.gimbalArm,...
            self.gimbalArm);
        
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
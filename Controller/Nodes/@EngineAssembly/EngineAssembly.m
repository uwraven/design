classdef EngineAssembly < handle

properties 
    gimbal
    thrust
    thrustRange
    specificThrust % inverse of thrust specific mass flow
    engineArm = 0.1;
    gimbalArm = 0.2;
end

properties (SetAccess = private, GetAccess = public)
    localizedResultant
    trilateration
end

properties (Access = private)
    allocation
end

methods
    function self = EngineAssembly()
        self.gimbal = Gimbal();
        self.localizedResultant = zeros(6, 1);
        self.thrustRange = [0 1];
    end

    function setAllocation(self, U)
        % U (Fe) is the linear approximation of engine forces requested by the allocator
        % Fex Fey Fez

        self.allocation = U;
        % allocation = U
 
        % First, convert to proper nonlinear values
        [Fe, g, b] = self.getLinearInputs(U);

        % Compute resultant engine pointing vector
        % Describes the physical position in space where linear
        % actuators connect to the engine
        % (ignores small radial distance from engine center)
        Ev = self.engineArm * [
            -sin(b) * cos(g)
            sin(g)
            cos(b) * cos(g)
        ];

        % enginePointingVector = Ev * Fe / self.engineArm;

        % Compute required actuator extensions from difference between
        % engine pointing vector and gimbal mounting vectors
        self.thrust = abs(Fe);
        self.gimbal.actuatorBeta.setReference(norm(Ev - [0 self.gimbalArm 0]'));
        self.gimbal.actuatorGamma.setReference(norm(Ev - [self.gimbalArm 0 0]'));
    end

    function update(self, dt)
        % Engine is assumed to produce thrust ideally with no delay
        % TODO:: Add engine plant model + thrust controller
        self.gimbal.update(dt);

        % Get engine direction from actuator extensions
        % This function does not account for the radial distance from
        % the lower engine mount to linear actuator

        % gx = self.gimbal.actuatorGamma.x
        % bx = self.gimbal.actuatorBeta.x

        [x y z] = trilaterate(self.engineArm, self.gimbal.actuatorGamma.x, self.gimbal.actuatorBeta.x, self.gimbalArm, 0, self.gimbalArm);

        thrustDirection = [x y -abs(z)]' / norm([x y z]);
        self.trilateration = [x y z];
        
        % Compute localized resultant from current thrust and gimbal trilateration
        localizedResultant = self.thrust .* [
            thrustDirection
            zeros(3, 1)
        ];
        % self.localizedResultant = localizedResultant;

        self.localizedResultant = self.allocation;

        % localizedError = self.localizedResultant(1:3) - self.allocation

    end

    function [Fe, gamma, beta] = getLinearInputs(self, requested)
        Fe = -requested(3);
        gamma = -requested(2) / Fe;
        beta = -requested(1) / Fe;
    end


    function [Fe, gamma, beta] = getNonlinearInputs(self, requested)
        % This function performs comparison of small numbers under certain circumstances and can lead
        % to erronous computation results
        beta = atansmooth(requested(1), requested(3));
        gamma = atansmooth(-requested(2) * sin(beta), requested(1));
        Fe = requested(2) / sin(gamma);
    end
    
end

end
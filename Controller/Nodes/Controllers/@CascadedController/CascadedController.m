classdef CascadedController < handle

properties
    trajectoryController = struct()
    rateController
    requestedAcceleration
    requestedAngularAcceleration
    reference = [0 0 -1]';
    enabled
end

methods
    function self = CascadedController()
        self.trajectoryController.x = PIDController(1, 0, 0);
        self.trajectoryController.y = PIDController(1, 0, 0);
        self.trajectoryController.z = PIDController(1, 0, 0);
        self.rateController = RateController();
    end

    function update(self, dt)
        % Do nothing for now
        if (self.enabled)
        end
    end

    function U = inputs(self, x, target, dt)
        if self.enabled
            % x: current state (R13)
            % target: reference position in 3D
            currentAttitude = x(7:10);
            currentAngularRate = x(11:13);
            
            % 1) Compute requested global accelerations given position error using decoupled PID controllers
            requestedAcceleration = [
                self.trajectoryController.x.inputs(x(1), target(1), dt)
                self.trajectoryController.y.inputs(x(2), target(2), dt)
                self.trajectoryController.z.inputs(x(3), target(3), dt)
            ];

            % 2) Compute rotation quaternion associated with requestedAcceleration
            requestedAccelerationQuat = Quaternion.fromDirection(requestedAcceleration, self.reference);

            % 3) Compute error between vehicle quaternion and requested acceleration quaternion
            requestedAttitudeError = Quaternion.error(requestedAccelerationQuat, currentAttitude);

            % 4) Compute angular acceleration using rate controller
            requestedAngularAcceleration = self.rateController.inputs(requestedAttitudeError(2:4), currentAngularRate);

            % Return global frame requests in 
            U = [
                requestedAcceleration
                requestedAngularAcceleration
            ];
        else
            U = [0 0 0 0 0 0]';
        end        
    end

end

end
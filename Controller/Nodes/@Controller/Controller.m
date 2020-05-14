classdef Controller < handle

properties
    trajectoryController
    rateController
    requestedAcceleration
    requestedAngularAcceleration
end

methods
    function self = Controller()
        trajectoryController = struct(...
            'x', PIDController(1, 0, 0),...
            'y', PIDController(1, 0, 0),...
            'z', PIDController(1, 0, 0)...
        );
        rateController = RateController();
    end

    function update(self, dt)
        % Do nothing for now
    end

    function U = inputs(self, x, target)
        % x: current state (R13)
        % target: reference position in 3D
        
        % 1) Compute requested global accelerations given position error using decoupled PID controllers

        % 2) Compute rotation quaternion associated with requestedAcceleration

        % 3) Compute error between vehicle quaternion and requested acceleration quaternion

        % 4) Compute angular acceleration using rate controller

        % Return compute global frame requests in U
        
    end

end

end
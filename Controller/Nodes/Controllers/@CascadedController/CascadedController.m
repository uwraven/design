classdef CascadedController < handle

properties
    trajectoryController
    rateController
    requestedAcceleration
    requestedAngularAcceleration
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
        % x: current state (R13)
        % target: reference position in 3D
        
        % 1) Compute requested global accelerations given position error using decoupled PID controllers
        

        % 2) Compute rotation quaternion associated with requestedAcceleration

        % 3) Compute error between vehicle quaternion and requested acceleration quaternion

        % 4) Compute angular acceleration using rate controller

        % Return global frame requests in U
        U = [0 0 0 0 0 0]';
        if (self.enabled)

        end
        
    end

end

end
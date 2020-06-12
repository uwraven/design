classdef CascadedController < handle

properties
    trajectoryController = struct()
    trajectoryPrefilter = struct()
    rateController
    requestedAcceleration
    requestedAngularAcceleration
    horizontalAccelerationLimit = 1;
    verticalAccelerationLimit = 4;
    reference = [0 0 -1]';
    enabled
    antiwindupEnabled

    % This is hack to allow the Cascaded Controller and LQR controller to work interchangeably
    independentPrefilter = true;
end

methods
    function self = CascadedController()
        self.trajectoryController.x = PIDController(1, 0, 0);
        self.trajectoryController.y = PIDController(1, 0, 0);
        self.trajectoryController.z = PIDController(1, 0, 0);
        self.trajectoryPrefilter.x = Prefilter();
        self.trajectoryPrefilter.y = Prefilter();
        self.trajectoryPrefilter.z = Prefilter();
        self.rateController = RateController();
    end

    function update(self, dt)
        % Do nothing for now
    end

    function U = inputs(self, x, target, ff, dt)
        if self.enabled
            % x: current state (R13)
            % target: reference position in 3D
            currentAttitude = x(7:10);
            currentAngularRate = x(11:13);

            self.trajectoryPrefilter.x.setTarget(target(1));
            self.trajectoryPrefilter.y.setTarget(target(2));
            self.trajectoryPrefilter.z.setTarget(target(3));

            self.trajectoryPrefilter.x.update(dt);
            self.trajectoryPrefilter.y.update(dt);
            self.trajectoryPrefilter.z.update(dt);

            % self.velocityPrefilter.x.update(dt);
            % self.velocityPrefilter.y.update(dt);
            % self.velocityPrefilter.z.update(dt);

            pf = [
                self.trajectoryPrefilter.x.output
                self.trajectoryPrefilter.y.output
                self.trajectoryPrefilter.z.output
                self.trajectoryPrefilter.x.outputDerivative;
                self.trajectoryPrefilter.y.outputDerivative;
                self.trajectoryPrefilter.z.outputDerivative;
            ];

            u = x(1:3) - pf(1:3);
            du = x(4:6) - pf(4:6);
            
            % 1) Compute requested global accelerations given position error using decoupled PID controllers
            requestedAcceleration = -[
                self.trajectoryController.x.inputs(x(1) - pf(1), x(4) - pf(4), dt)
                self.trajectoryController.y.inputs(x(2) - pf(2), x(5) - pf(5), dt)
                self.trajectoryController.z.inputs(x(3) - pf(3), x(6) - pf(6), dt)
            ];

            requestedAcceleration = requestedAcceleration + ff;

            if (self.antiwindupEnabled)
                if abs(requestedAcceleration(1)) > self.horizontalAccelerationLimit
                    requestedAcceleration(1) = requestedAcceleration(1) + self.trajectoryController.x.err_i * self.trajectoryController.x.ki;
                end
                if abs(requestedAcceleration(2)) > self.horizontalAccelerationLimit
                    requestedAcceleration(2) = requestedAcceleration(2) + self.trajectoryController.y.err_i * self.trajectoryController.y.ki;
                end
                if abs(requestedAcceleration(3) - ff(3)) > self.verticalAccelerationLimit
                    requestedAcceleration(3) = requestedAcceleration(3) + self.trajectoryController.z.err_i * self.trajectoryController.z.ki;
                end
            end

            % 2) Compute rotation quaternion associated with requestedAcceleration
            requestedAccelerationQuat = Quaternion.fromDirection(requestedAcceleration, self.reference);

            % 3) Compute error between vehicle quaternion and requested acceleration quaternion
            requestedAttitudeError = Quaternion.error(currentAttitude, requestedAccelerationQuat);

            % 4) Compute angular acceleration using rate controller
            requestedAngularAcceleration = self.rateController.inputs(requestedAttitudeError(2:4), currentAngularRate);

            % Return global frame requests
            U = [
                requestedAcceleration
                requestedAngularAcceleration
            ];
        else
            U = [0 0 0 0 0 0]';
        end        
    end

end

methods (Access = private)
    
end

end
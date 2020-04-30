classdef PIDController < handle

properties (GetAccess = public, SetAccess = private)
    kp = 1;
    ki = 0
    kd = 0
end

properties (Access = private)
    uPrevious
    err_d
    err_i
    derivativeTrackingFrequency = 0; % Derivative tracking
end

methods (Access = public)

    function self = PIDController(kp, ki, kd)
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    end

    function U = update(self, dt, u, saturated)
        % Update the error rate of change
        err_d = (u - self.uPrevious) / dt;
        a = dt / (self.derivativeTrackingFrequency + dt);
        self.err_d = self.err_d * (1 - a) + err_d * a;

        % Update integral error only if controls are not saturated
        if (~saturated)
            self.err_i = self.err_i + u * dt;
        end

        % Compute the controller output for time k
        U = self.kp * u + self.kd * self.err_d + self.ki * self.err_i;

        % Update the error to compute derivative at time k+1
        self.uPrevious = u;
    end

    function setGains(self, kp, ki, kd)
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    end

end
    
end
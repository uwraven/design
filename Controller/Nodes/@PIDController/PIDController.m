classdef PIDController < handle

properties (Access = public)
    
end

properties (Access = private)
    kp
    ki
    kd
    err_prev
    err_d
    err_i
    A = 0; % Premultiplier used for tuning around system properties
    cf = 1; % Derivative tracking
end

methods (Access = public)

    function self = PIDController(kp, ki, kd)
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    end

    function U = tick(self, dt, u, saturated)
        % Update the error rate of change
        err_d = (u - self.err_prev) / dt;
        self.err_d = self.err_d * (1 - self.cf) + err_d * self.cf;

        % Update integral error only if controls are not saturated
        if (~saturated)
            self.err_i = self.err_i + u * dt;
        end

        % Compute the controller output for time k
        U = self.kp * u + self.kd * self.err_d + self.ki * self.err_i;

        % Update the error to compute derivative at time k+1
        self.err_prev = u;
    end

    function setGains(self, kp, ki, kd)
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    end

    function setPremultiplier(self, A)
        self.A = A;
    end

    function setDerivativeTracking(self, cf)
        self.cf = cf;
    end

    function K = getGains(self)
        K = [self.kp, self.ki, self.kd];
    end

end
    
end
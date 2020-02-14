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
end

methods (Access = public)

    function self = PIDController(kp, ki, kd)
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    end

    function U = tick(self, dt, u)
        % Update the error rate of change and error integral terms for time k
        self.err_d = (u - self.err_prev) / dt;
        self.err_i = self.err_i + u * dt;

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

    function K = getGains(self)
        K = [self.kp, self.ki, self.kd];
    end

end
    
end
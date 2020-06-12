classdef PIDController < handle

properties (GetAccess = public, SetAccess = private)
    kp = 1
    ki = 0
    kd = 0
    Kt = 0
    u_prev = 0;
end

properties
    err_d = 0;
    err_i = 0;
end

methods (Access = public)

    function self = PIDController(kp, ki, kd)
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    end

    function update(self, dt)

    end

    function U = inputs(self, u, du, dt)

        err_d = 0;

        err_i = self.err_i + u * dt;

        % Compute the controller output for time k
        U = self.kp * u + self.kd * du + self.ki * err_i;

        % Update the error to compute derivative at time k+1
        self.u_prev = u;
        self.err_d = du;
        self.err_i = err_i;
    end

    function setGains(self, kp, ki, kd, Kt)
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
        self.Kt = Kt;
    end

end
    
end
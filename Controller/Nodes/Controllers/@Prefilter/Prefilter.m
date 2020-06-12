classdef Prefilter < handle

properties (Access = public)
    target
    output
    outputDerivative
    x
    dx
    u
    du
    enabled = true
end

properties (Access = private)
    sys
    dsys
    A
    B
    C
end

methods

    function self = Prefilter()
        self.target = 0;
        self.output = 0;
    end

    function update(self, dt)
        dx = self.A * self.x + self.B * self.target;
        self.x = self.x + dx * dt;
        self.output = self.C * self.x;
        self.outputDerivative = self.C * dx;
        if (~self.enabled)
            self.output = self.target
            self.outputDerivative = 0;
        end
    end

    function setTarget(self, target)
        self.target = target;
    end

    function setInitialValue(self, x)
        self.x(1) = x;
    end

    function setGains(self, wn, dr, kp, ki, kd)
        s = tf('s');
        C = kp + ki / s + kd * s;
        P = 1 / s^2;
        L = C * P;
        T = wn^2 / (s^2 + 2 * dr * wn * s + wn^2);
        F = (1 + L) * T / L;
        sys = ss(F);
        self.dsys = ss(F * s);
        self.A = sys.A;
        self.B = sys.B;
        self.C = sys.C;
        self.x = zeros(length(sys.A(1,:)), 1);
        self.u = zeros(length(sys.B(1,:)), 1);
        self.dx = zeros(length(self.dsys.A(1,:)), 1);
        self.du = zeros(length(sys.B(1,:)), 1);
        self.sys = sys;
    end

end

end
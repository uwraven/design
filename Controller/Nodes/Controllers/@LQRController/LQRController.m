classdef LQRController < handle
    properties (Access = public)

    end

    properties (SetAccess = private, GetAccess = public)
        K
    end

    methods (Access = public)
        function self = LQRController()
        end

        function update(self, dt)

        end

        function U = inputs(self, x, target, dt)
            % TODO: input shape validation
            trajectory_error = target - x(1:3);
            U = self.K * [trajectory_error; x(4:6); x(8:end)];
        end

        function setGains(self, K)
            self.K = K;
        end
    end

end
classdef LQRController < handle
    properties (Access = public)
        enabled = true;
        prefilterCutoffFrequency = 0;
        filteredCoordinate = [0 0 0]';

        % This is a hack to get LQR Controller and CascadedController to work interchangeably 
        independentPrefilter = false;
    end

    properties (SetAccess = private, GetAccess = public)
        K
    end

    methods (Access = public)
        function self = LQRController()
        end

        function update(self, dt)
        end

        function U = inputs(self, x, target, ff, dt)
            % TODO: input shape validation
            a = 2 * pi * dt * self.prefilterCutoffFrequency / (2 * pi * dt * self.prefilterCutoffFrequency + 1);
            self.filteredCoordinate = target * a + self.filteredCoordinate * (1 - a);

            if self.enabled
                trajectory_error = x(1:3) - self.filteredCoordinate;
                U = -self.K * [trajectory_error; x(4:6); x(8:end)] + [ ff; zeros(3, 1) ];
            else
                U = zeros(6, 1);
            end
        end

        function setGains(self, K)
            self.K = K;
        end
    end

end
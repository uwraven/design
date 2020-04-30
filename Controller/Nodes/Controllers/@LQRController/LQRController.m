classdef LQRController < handle
    properties (Access = public)

    end

    properties (SetAccess = private, GetAccess = public)
        K
    end

    methods (Access = public)
        function self = LQRController()
        end

        function U = inputs(self, X)
            % TODO: input shape validation
            U = self.K * X;
        end

        function setGains(self, K)
            self.K = K;
        end
    end

end